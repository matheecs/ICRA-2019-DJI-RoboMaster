//
// Created by cxn on 19-3-1.
//

#ifndef PROJECT_WRITETXT_H
#define PROJECT_WRITETXT_H
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>



//
//int main()
//{
//using namespace std;
//    // 写文件
//    ofstream outFile;
//    outFile.open("data.csv", ios::out); // 打开模式可省略
//    outFile << "name" << ',' << "age" << ',' << "hobby" << endl;
//    outFile << "Mike" << ',' << 18 << ',' << "paiting" << endl;
//    outFile << "Tom" << ',' << 25 << ',' << "football" << endl;
//    outFile << "Jack" << ',' << 21 << ',' << "music" << endl;
//    outFile.close();
//
//    // 读文件
//    ifstream inFile("data.csv", ios::in);
//    string lineStr;
//    vector<vector<string>> strArray;
//    while (getline(inFile, lineStr))
//    {
//        // 打印整行字符串
//        cout << lineStr << endl;
//        // 存成二维表结构
//        stringstream ss(lineStr);
//        string str;
//        vector<string> lineArray;
//        // 按照逗号分隔
//        while (getline(ss, str, ','))
//            lineArray.push_back(str);
//        strArray.push_back(lineArray);
//    }
//
//    getchar();
//    return 0;
//}


namespace leonard_serial_common {
    class CsvWriter {
    public:
        CsvWriter(std::string path, std::vector<std::string> topic) {
            name_ = path;
            outFile_.open(path, std::ios::out); // 打开模式可省略
            int i;
            for (i = 0; i < topic.size() - 1; i++) {
                outFile_ << topic[i] << ',';
            }
            outFile_ << topic[i] << std::endl;
            topic_size_ = topic.size();
            count_ = 0;
        }

        ~CsvWriter() {
            outFile_.close();
        }

        template<typename T>
        void write(T data) {
            outFile_ << data;
            if (++count_ % topic_size_ != 0) {
                outFile_ << ',';
            } else {
                outFile_ << std::endl;
            }
        }

    private:
        std::string name_;
        std::ofstream outFile_;
        int topic_size_;
        int64_t count_;
    };
}
#endif //PROJECT_WRITETXT_H
