// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include <cstdint>
#include <cstring>
#include <memory>
#include <array>
#include <opencv2/flann/lsh_table.h>
#include <opencv2/flann/result_set.h>

#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
// Same Hamming metric as OpenCV's byte lookup table. Only this function requires
// POPCNT; index construction selects it once after checking the host CPU.
struct RiftSOrbHamming : cvflann::HammingLUT
{
	using cvflann::HammingLUT::operator();
	__attribute__((target("popcnt"))) int
	operator()(const unsigned char *a, const unsigned char *b, size_t size) const
	{
		if (size != 32)
			return cvflann::HammingLUT::operator()(a, b, size);
		int distance = 0;
		for (size_t word = 0; word < 4; ++word) {
			uint64_t first, second;
			std::memcpy(&first, a + word * 8, 8);
			std::memcpy(&second, b + word * 8, 8);
			distance += __builtin_popcountll(first ^ second);
		}
		return distance;
	}
};
#endif

#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
// OpenCV builds the same randomized tables. A presence bitset and direct ranges
// avoid hash lookups and bucket pointer chasing during each table's 211 probes.
class RiftSOrbIndex
{
public:
	explicit RiftSOrbIndex(const cv::Mat &descriptors) : train(descriptors.clone())
	{
		cvflann::Matrix<unsigned char> dataset(train.data, train.rows, train.cols, train.step);
		masks.push_back(0);
		for (int high = 19; high >= 0; --high) {
			masks.push_back(uint32_t(1) << high);
			for (int low = high - 1; low >= 0; --low)
				masks.push_back((uint32_t(1) << high) | (uint32_t(1) << low));
		}
		for (auto &table : tables) {
			table.hash = cvflann::lsh::LshTable<unsigned char>(32, 20);
			table.hash.add(dataset);
			table.buckets.resize(size_t(1) << 20);
			table.present.resize(size_t(1) << 14);
			table.candidates.reserve(size_t(train.rows) * 2);
			for (int row = 0; row < train.rows; ++row) {
				auto key = table.hash.getKey(train.ptr<unsigned char>(row));
				auto bit = uint64_t(1) << (key & 63);
				if (table.present[key >> 6] & bit)
					continue;
				table.present[key >> 6] |= bit;
				const auto &bucket = *table.hash.getBucketFromKey(key);
				table.buckets[key] = uint32_t(table.candidates.size());
				table.candidates.push_back(uint32_t(bucket.size()));
				table.candidates.insert(table.candidates.end(), bucket.begin(), bucket.end());
			}
		}
	}

	void
	knnMatch(const cv::Mat &queries, std::vector<std::vector<cv::DMatch>> &nearest, int count) const
	{
		CV_Assert(queries.type() == CV_8U && queries.cols == 32 && count > 0 && count <= train.rows);
		nearest.clear();
		nearest.resize(queries.rows);
		cvflann::KNNUniqueResultSet<int> result(count);
		std::vector<int> indices(count), distances(count);
		RiftSOrbHamming distance;
		for (int row = 0; row < queries.rows; ++row) {
			result.clear();
			const auto *query = queries.ptr<unsigned char>(row);
			for (const auto &table : tables) {
				auto key = table.hash.getKey(query);
				for (auto mask : masks) {
					auto probe = key ^ mask;
					if (!(table.present[probe >> 6] & (uint64_t(1) << (probe & 63))))
						continue;
					auto start = table.buckets[probe];
					auto end = start + 1 + table.candidates[start];
					for (uint32_t at = start + 1; at < end; ++at) {
						auto candidate = table.candidates[at];
						result.addPoint(
						    distance(query, train.ptr<unsigned char>(candidate), 32),
						    candidate);
					}
				}
			}
			std::fill(indices.begin(), indices.end(), -1);
			result.sortAndCopy(indices.data(), distances.data(), count);
			nearest[row].reserve(count);
			for (int neighbor = 0; neighbor < count; ++neighbor)
				if (indices[neighbor] >= 0)
					nearest[row].emplace_back(row, indices[neighbor], 0,
					                          float(distances[neighbor]));
		}
	}

private:
	struct Table
	{
		cvflann::lsh::LshTable<unsigned char> hash;
		std::vector<uint32_t> buckets;
		std::vector<uint64_t> present;
		std::vector<uint32_t> candidates;
	};
	cv::Mat train;
	std::array<Table, 12> tables;
	std::vector<uint32_t> masks;
};
#endif

// Keeps the original BF/LSH policy, search parameters and result order. The
// public FLANN tables retain the original randomized candidate selection.
class RiftSWorldMatcher
{
public:
	explicit RiftSWorldMatcher(const cv::Mat &descriptors)
	{
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
		if (descriptors.rows > 2000 && __builtin_cpu_supports("popcnt")) {
			fast = std::make_unique<RiftSOrbIndex>(descriptors);
			return;
		}
#endif
		if (descriptors.rows > 2000)
			fallback =
			    cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
		else
			fallback = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING);
		fallback->add(std::vector<cv::Mat>{descriptors});
		fallback->train();
	}

	void
	knnMatch(const cv::Mat &descriptors, std::vector<std::vector<cv::DMatch>> &nearest, int count)
	{
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
		if (fast) {
			// OpenCV hashes rows through size_t loads. A packed aligned copy
			// keeps sliced or unaligned callers out of that loader.
			cv::Mat queries = descriptors;
			if (!queries.isContinuous() || reinterpret_cast<uintptr_t>(queries.data) % alignof(size_t))
				queries = descriptors.clone();
			fast->knnMatch(queries, nearest, count);
			return;
		}
#endif
		fallback->knnMatch(descriptors, nearest, count);
	}

private:
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
	std::unique_ptr<RiftSOrbIndex> fast;
#endif
	cv::Ptr<cv::DescriptorMatcher> fallback;
};
