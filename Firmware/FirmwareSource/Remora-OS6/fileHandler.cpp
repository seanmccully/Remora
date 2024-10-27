#include "fileHandler.h"

FileHandler::FileHandler(const std::string& filename) 
        : _file(filename) 
    { 
			fileSize(); 
		}

bool FileHandler::fileOpen() {
	return _file.is_open();
}

void FileHandler::fileSize() {

    if (!_file.is_open()) {
    		fileEnd = 0; 
				return;
    }

		// Seek to the end of the file
    _file.seekg(0, std::ios::end);

    // Get the current position, which is the file size
    fileEnd = _file.tellg();
		_file.seekg(0, ios::beg);
}

std::string FileHandler::readFile() {
    // Read file in chunks
		if (!fileOpen())
			return string("");

    string buffer = "";
		buffer.resize(fileEnd);
		_file.read(&buffer[0], fileEnd);
		_file.seekg(0, std::ios::beg);
		
		return buffer;
}

FileHandler::~FileHandler() {
    if (_file.is_open()) {
            _file.close();
        }
}
