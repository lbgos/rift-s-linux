// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include <cstdint>
#include <cstring>
#include <memory>

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

// Keeps the original BF/LSH policy, search parameters and result order. The
// templated public FLANN API lets the native driver supply a faster distance.
class RiftSWorldMatcher
{
public:
	explicit RiftSWorldMatcher(const cv::Mat &descriptors)
	{
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
		if (descriptors.rows > 2000 && __builtin_cpu_supports("popcnt")) {
			fast = std::make_unique<cv::flann::GenericIndex<RiftSOrbHamming>>(
			    descriptors, cvflann::LshIndexParams(12, 20, 2));
			return;
		}
#endif
		if (descriptors.rows > 2000)
			fallback = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
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
			// GenericIndex requires continuous query storage. ORB is normally
			// continuous, but a sliced query must keep the same search behavior.
			cv::Mat queries = descriptors.isContinuous() ? descriptors : descriptors.clone();
			cv::Mat indices(queries.rows, count, CV_32S), distances(queries.rows, count, CV_32S);
			fast->knnSearch(queries, indices, distances, count, cvflann::SearchParams(32, 0, true));
			nearest.clear();
			nearest.resize(queries.rows);
			for (int row = 0; row < queries.rows; ++row) {
				nearest[row].reserve(count);
				for (int neighbor = 0; neighbor < count; ++neighbor) {
					int index = indices.at<int>(row, neighbor);
					if (index >= 0)
						nearest[row].emplace_back(row, index, 0, float(distances.at<int>(row, neighbor)));
				}
			}
			return;
		}
#endif
		fallback->knnMatch(descriptors, nearest, count);
	}

private:
#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
	std::unique_ptr<cv::flann::GenericIndex<RiftSOrbHamming>> fast;
#endif
	cv::Ptr<cv::DescriptorMatcher> fallback;
};
