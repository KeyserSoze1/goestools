#pragma once

#include <string>
#include <vector>

struct Options {
  std::string nanomsg;
  std::vector<std::string> files;
  bool dryrun = false;

  // File types to include
  bool images = false;
  bool messages = false;
  bool text = false;
  bool dcs = false;
  bool emwin = false;
};

Options parseOptions(int argc, char** argv);
