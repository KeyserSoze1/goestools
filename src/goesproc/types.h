#pragma once

#include <functional>
#include <string>
#include <tuple>
#include <vector>

// AWIPS product identifier
struct AWIPS {
  std::string t1t2;
  std::string a1a2;
  std::string ii;
  std::string cccc;
  std::string yy;
  std::string gggg;
  std::string bbb;
  std::string nnn;
  std::string xxx;
};

// Image region (e.g. FD/Full Disk or NH/Northern Hemisphere)
struct Region {
  std::string nameShort;
  std::string nameLong;
};

// Image channel (e.g. IR/Infrared or CH02/Channel 02)
struct Channel {
  std::string nameShort;
  std::string nameLong;
};

// Used to key a vector of segments off of its region and channel
using SegmentKey = std::tuple<std::string, std::string>;

// Corresponding hash function
struct SegmentKeyHash : public std::unary_function<SegmentKey, std::size_t> {
  std::size_t operator()(const SegmentKey& k) const {
    return
      std::hash<std::string>()(std::get<0>(k)) ^
      std::hash<std::string>()(std::get<1>(k));
  }
};
