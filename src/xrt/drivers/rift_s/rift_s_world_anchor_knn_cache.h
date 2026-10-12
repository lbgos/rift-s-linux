// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

// Bounded EXACT memoization for the room relocalization knnMatch path.
//
// Soundness: knnMatch over a fixed, trained matcher is a deterministic function of the
// 32-byte query descriptor bytes plus the unchanged index state (same train data, same
// LSH/BF params, same k=4, no masks). Each cached entry stores the full k-neighbor list in
// returned order, including empty lists, and re-stamps queryIdx to the caller's row on hits.
// Batched misses go through one knnMatch call so per-row tie ordering and distances are
// identical to an uncached call; hits only skip redundant searches.
//
// Invalidation: the owner clears both bank caches on every index rebuild and whenever the
// map is replaced/cleared/reloaded (anything that sets index_dirty). Entries are keyed by
// exact 32-byte equality, never by hash approximation.
//
// Bounds: 8192 entries per bank, FIFO eviction. Each entry holds at most 4 neighbors.

#include <opencv2/features2d.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <unordered_map>
#include <vector>

namespace rift_s_knn_memo {

struct DescriptorKey
{
	uint64_t w[4] = {0, 0, 0, 0};

	bool
	operator==(const DescriptorKey &o) const noexcept
	{
		return w[0] == o.w[0] && w[1] == o.w[1] && w[2] == o.w[2] && w[3] == o.w[3];
	}
};

struct DescriptorKeyHash
{
	size_t
	operator()(const DescriptorKey &k) const noexcept
	{
		// FNV-1a over the 32 bytes, then a short avalanche for table spread.
		uint64_t h = 14695981039346656037ULL;
		for (int i = 0; i < 4; ++i) {
			uint64_t v = k.w[i];
			for (int b = 0; b < 8; ++b) {
				h ^= uint8_t(v >> (8 * b));
				h *= 1099511628211ULL;
			}
		}
		h ^= h >> 29;
		h *= 0xbf58476d1ce4e5b9ULL;
		h ^= h >> 27;
		h *= 0x94d049bb133111ebULL;
		h ^= h >> 31;
		return size_t(h);
	}
};

inline DescriptorKey
make_key(const uint8_t *d)
{
	DescriptorKey k;
	std::memcpy(k.w, d, 32);
	return k;
}

// Stored k-neighbor list in matcher return order. queryIdx is intentionally not stored:
// it is re-stamped to the caller's row on every hit.
struct CachedNeighbors
{
	uint8_t count = 0;
	int trainIdx[4] = {-1, -1, -1, -1};
	float distance[4] = {0, 0, 0, 0};
	int imgIdx[4] = {0, 0, 0, 0};
};

class ExactKnnMemo
{
public:
	static constexpr size_t kMaxEntries = 8192;
	static constexpr int kNeighbors = 4;

	ExactKnnMemo()
	{
		map_.reserve(kMaxEntries * 2);
	}

	void
	clear()
	{
		map_.clear();
		fifo_.clear();
	}

	size_t
	size() const
	{
		return map_.size();
	}

	bool
	lookup(const DescriptorKey &key, CachedNeighbors &out) const
	{
		auto it = map_.find(key);
		if (it == map_.end())
			return false;
		out = it->second;
		return true;
	}

	// Insert a matcher-returned list. Duplicate keys keep their original FIFO slot.
	void
	insert(const DescriptorKey &key, const std::vector<cv::DMatch> &list)
	{
		if (map_.find(key) != map_.end())
			return;
		if (map_.size() >= kMaxEntries && !fifo_.empty()) {
			map_.erase(fifo_.front());
			fifo_.pop_front();
		}
		CachedNeighbors e;
		size_t n = list.size() < 4 ? list.size() : 4;
		e.count = uint8_t(n);
		for (size_t i = 0; i < n; ++i) {
			e.trainIdx[i] = list[i].trainIdx;
			e.distance[i] = list[i].distance;
			e.imgIdx[i] = list[i].imgIdx;
		}
		map_.emplace(key, e);
		fifo_.push_back(key);
	}

	// Exact memoized knnMatch with k == kNeighbors. On any unexpected query layout the
	// call falls back to a direct uncached knnMatch so behavior never changes.
	template <typename Matcher>
	void
	knnMatch(Matcher &matcher,
	         const cv::Mat &query,
	         std::vector<std::vector<cv::DMatch>> &out,
	         int k = kNeighbors)
	{
		if (query.rows == 0 || query.data == nullptr) {
			out.clear();
			return;
		}
		if (query.type() != CV_8U || query.cols != 32 || k != kNeighbors) {
			matcher.knnMatch(query, out, k);
			return;
		}
		out.resize(size_t(query.rows));
		std::vector<DescriptorKey> keys(size_t(query.rows));
		std::vector<int> misses;
		misses.reserve(size_t(query.rows));
		for (int i = 0; i < query.rows; ++i) {
			keys[size_t(i)] = make_key(query.ptr<uint8_t>(i));
			auto it = map_.find(keys[size_t(i)]);
			if (it == map_.end()) {
				misses.push_back(i);
				continue;
			}
			const CachedNeighbors &e = it->second;
			out[size_t(i)].clear();
			out[size_t(i)].reserve(e.count);
			for (uint8_t j = 0; j < e.count; ++j)
				out[size_t(i)].emplace_back(i, e.trainIdx[j], e.imgIdx[j], e.distance[j]);
		}
		if (misses.empty())
			return;
		cv::Mat miss_mat(int(misses.size()), 32, CV_8U);
		for (size_t j = 0; j < misses.size(); ++j)
			std::memcpy(miss_mat.ptr<uint8_t>(int(j)), query.ptr<uint8_t>(misses[j]), 32);
		std::vector<std::vector<cv::DMatch>> miss_out;
		matcher.knnMatch(miss_mat, miss_out, k);
		// With no masks and compactResult=false the matcher returns one list per query row.
		if (miss_out.size() != misses.size()) {
			// Defensive: never fabricate neighbors; fall back to a direct full query so the
			// caller still sees exact uncached semantics, then refresh the cache from it.
			matcher.knnMatch(query, out, k);
			for (int i = 0; i < query.rows; ++i) {
				if (size_t(i) >= out.size())
					break;
				insert(keys[size_t(i)], out[size_t(i)]);
				for (auto &m : out[size_t(i)])
					m.queryIdx = i;
			}
			return;
		}
		for (size_t j = 0; j < misses.size(); ++j) {
			int row = misses[j];
			out[size_t(row)] = miss_out[j];
			for (auto &m : out[size_t(row)])
				m.queryIdx = row;
			insert(keys[size_t(row)], out[size_t(row)]);
		}
	}

private:
	std::unordered_map<DescriptorKey, CachedNeighbors, DescriptorKeyHash> map_;
	std::deque<DescriptorKey> fifo_;
};

} // namespace rift_s_knn_memo
