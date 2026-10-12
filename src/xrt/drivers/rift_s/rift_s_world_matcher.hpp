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
// OpenCV builds the same randomized tables. Direct bucket pointers remove hash
// lookups during the 211 probes per table without changing candidate order.
class RiftSOrbIndex
{
public:
	explicit RiftSOrbIndex(const cv::Mat &descriptors) : train(descriptors)
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
			table.buckets.resize(size_t(1) << 20, nullptr);
			for (int row = 0; row < train.rows; ++row) {
				auto key = table.hash.getKey(train.ptr<unsigned char>(row));
				table.buckets[key] = table.hash.getBucketFromKey(key);
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
					const auto *bucket = table.buckets[key ^ mask];
					if (bucket == nullptr)
						continue;
					for (auto candidate : *bucket)
						result.addPoint(
						    distance(query, train.ptr<unsigned char>(candidate), 32),
						    candidate);
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
		std::vector<const cvflann::lsh::Bucket *> buckets;
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
			fast->knnMatch(descriptors, nearest, count);
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
