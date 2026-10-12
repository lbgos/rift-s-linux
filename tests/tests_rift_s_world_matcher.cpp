// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_world_matcher.hpp"
#include <opencv2/flann/random.h>

TEST_CASE("Native room descriptor distance preserves BF and LSH neighbor identities")
{
	for (int rows : {64, 2000, 2400, 12000}) {
		cv::Mat train(rows, 32, CV_8U), queries(128, 32, CV_8U);
		cv::RNG random(19);
		random.fill(train, cv::RNG::UNIFORM, 0, 256);
		random.fill(queries, cv::RNG::UNIFORM, 0, 256);
		// Include equal-distance duplicates and sparse LSH queries with fewer
		// than four neighbors. Approximate candidate selection must not change.
		for (int row = 0; row < 32; ++row) {
			train.row(row).copyTo(queries.row(row));
			train.row(row).copyTo(train.row(row + 32));
		}
		for (unsigned seed : {3, 42, 97}) {
			cvflann::seed_random(seed);
			cv::Ptr<cv::DescriptorMatcher> reference;
			if (rows > 2000)
				reference = cv::makePtr<cv::FlannBasedMatcher>(
				    cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
			else
				reference = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING);
			reference->add(std::vector<cv::Mat>{train});
			reference->train();
			cvflann::seed_random(seed);
			RiftSWorldMatcher native(train);
			// Non-contiguous rows must retain query indices and neighbor order.
			cv::Mat padded(queries.rows, 35, CV_8U);
			cv::Mat sliced = padded.colRange(1, 33);
			queries.copyTo(sliced);
			for (const cv::Mat &query : {queries, sliced})
				for (int count : {1, 4, 8}) {
					std::vector<std::vector<cv::DMatch>> expected, actual;
					reference->knnMatch(query.clone(), expected, count);
					native.knnMatch(query, actual, count);
					REQUIRE(actual.size() == expected.size());
					for (size_t row = 0; row < expected.size(); ++row) {
						REQUIRE(actual[row].size() == expected[row].size());
						for (size_t neighbor = 0; neighbor < expected[row].size(); ++neighbor) {
							const auto &a = actual[row][neighbor],
							           &b = expected[row][neighbor];
							CHECK(a.queryIdx == b.queryIdx);
							CHECK(a.trainIdx == b.trainIdx);
							CHECK(a.imgIdx == b.imgIdx);
							CHECK(a.distance == b.distance);
						}
					}
				}
		}
	}
}

#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
TEST_CASE("Native ORB Hamming popcount handles unaligned words and generic tails")
{
	if (!__builtin_cpu_supports("popcnt"))
		SKIP("Host does not support POPCNT");
	unsigned char first[66], second[66];
	cv::RNG random(13);
	for (size_t i = 0; i < sizeof(first); ++i) {
		first[i] = uint8_t(random.next());
		second[i] = uint8_t(random.next());
	}
	RiftSOrbHamming native;
	cvflann::HammingLUT reference;
	for (size_t count : {0, 1, 7, 8, 31, 32, 33, 64})
		CHECK(native(first + 1, second + 1, count) == reference(first + 1, second + 1, count));
}
#endif
