#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

class FileHandler {
public:
    FileHandler(const string& filename);
    ~FileHandler();
    ifstream& get() { return _file; }
		bool fileOpen();
		void fileSize();
		string readFile();

private:
    ifstream _file;
	  streampos fileEnd;
};
