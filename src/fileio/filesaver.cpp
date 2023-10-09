#include "filesaver.h"

#include <absl/strings/str_format.h>

FileSaver::FileSaver(const string &filename, int columns, int filetype) {
    open(filename, columns, filetype);
}

bool FileSaver::open(const string &filename, int columns, int filetype) {
    auto type = filetype == TEXT ? std::ios_base::out : (std::ios_base::out | std::ios_base::binary);
    filefp_.open(filename, type);

    columns_  = columns;
    filetype_ = filetype;

    return isOpen();
}

void FileSaver::dump(const vector<double> &data) {
    dump_(data);
}

void FileSaver::dumpn(const vector<vector<double>> &data) {
    for (const auto &k : data) {
        dump_(k);
    }
}

void FileSaver::dump_(const vector<double> &data) {
    if (filetype_ == TEXT) {
        string line;

        constexpr absl::string_view format = "%-15.9lf ";

        line = absl::StrFormat(format, data[0]);
        for (size_t k = 1; k < data.size(); k++) {
            absl::StrAppendFormat(&line, format, data[k]);
        }

        filefp_ << line << "\n";
    } else {
        filefp_.write(reinterpret_cast<const char *>(data.data()), sizeof(double) * data.size());
    }
}
