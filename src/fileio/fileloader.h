#ifndef FILELOADER_H
#define FILELOADER_H

#include "filebase.h"

class FileLoader : public FileBase {

public:
    FileLoader() = default;
    FileLoader(const string &filename, int columns, int filetype = TEXT);

    bool open(const string &filename, int columns, int filetype = TEXT);

    vector<double> load();
    vector<vector<double>> loadn(int epochs);

    bool load(vector<double> &data);
    bool loadn(vector<vector<double>> &data, int epochs);

private:
    vector<double> data_;

    bool load_();
};

#endif // FILELOADER_H
