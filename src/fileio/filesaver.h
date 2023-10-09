#pragma once

#include <absl/strings/string_view.h>

#include <fstream>
#include <string>
#include <vector>

#include "filebase.h"

class FileSaver : public FileBase {
 public:
  FileSaver() = default;
  FileSaver(const string &filename, int columns, int filetype = TEXT);

  bool open(const string &filename, int columns, int filetype = TEXT);

  void dump(const vector<double> &data);
  void dumpn(const vector<vector<double>> &data);

 private:
  void dump_(const vector<double> &data);
};