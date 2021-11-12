//
// Created by yc on 2021/9/3.
//

#ifndef CSVREADER_CSVREADER_H
#define CSVREADER_CSVREADER_H

#include <fstream>
#include <string>
#include <sstream>


class csvReader{
public:
    csvReader(const char *);
    int readLine();
    int data[12];
private:
    std::ifstream _csvInput;
};

csvReader::csvReader(const char * path) {
    _csvInput.open(path);
}

int csvReader::readLine() {
    std::string _oneLine;
    getline(_csvInput,_oneLine);
    if (_oneLine[0] == 'r')
        return EXIT_FAILURE;

    std::istringstream _readStr(_oneLine);
    std::string partOfstr;
    for (int i = 0; i < 12; i++) {
        getline(_readStr,partOfstr,',');
        data[i] = atoi(partOfstr.c_str());
    }

    if ((data[0] || data[1] || data[2]) == 0)
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}


#endif //CSVREADER_CSVREADER_H
