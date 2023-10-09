#pragma once

#include <fstream>
#include <vector>

using std::string;
using std::vector;

class FileBase {
 public:
  static const int TEXT = 0;
  static const int BINARY = 1;

  FileBase() = default;
  ~FileBase() {
    if (isOpen()) filefp_.close();
  }

  void close() { filefp_.close(); }

  bool isOpen() { return filefp_.is_open(); }

  bool isEof() { return filefp_.eof(); }

  std::fstream &fstream() { return filefp_; }

  int columns() const { return columns_; }

 protected:
  std::fstream filefp_;
  int filetype_ = TEXT;

  int columns_ = 0;
};