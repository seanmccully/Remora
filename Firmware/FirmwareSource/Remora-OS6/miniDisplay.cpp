// miniDisplay.cpp

#include "miniDisplay.h"


// miniDisplay.cpp - Font Data
// First, convert the Python byte strings to C++ array

miniDisplay::miniDisplay(
    PinName spiMosi, PinName spiSck,
    PinName lcdRs, PinName csPin, PinName resetPin
) :
    _spi(spiMosi, NC, spiSck),
    _rs(lcdRs),
    _cs(csPin),
    _reset(resetPin)
{
    _spi.format(8, 0);         // 8 bits, mode 0
    _spi.frequency(10000000);  // 10MHz
    // Clear buffers
    memset(_vram, 0, sizeof(_vram));
    memset(_old_vram, 0, sizeof(_old_vram));
    
}

void miniDisplay::init() {
    // Reset display
    _reset = 0;
    wait_us(100000);  // 100ms
    _reset = 1;
    wait_us(100000);  // 100ms
    
    // Initialize UC1701
    const uint8_t init_sequence[] = {
        0xE2,  // System reset
        0x40,  // Set display start line to 0
        0xA0,  // Set SEG direction
        0xC8,  // Set COM Direction
        0xA2,  // Set LCD bias to 1/9
        0x2C,  // Boost ON
        0x2E,  // Voltage regulator on
        0x2F,  // Voltage follower on
        0xF8,  // Set booster ratio
        0x00,  // Booster ratio value (4x)
        0x23,  // Set resistor ratio (3)
        0x81,  // Set Electronic Volume
        0x28,  // Electronic Volume value (contrast)
        0xAC,  // Set static indicator off
        0x00,  // NOP
        0xA6,  // Disable Inverse
        0xAF   // Display ON
    };
    
    for (auto cmd : init_sequence) {
        writeCommand(cmd);
    }
    
    setContrast(55);
    // Clear display
    clear();
}

void miniDisplay::writeCommand(uint8_t cmd) {
    _cs = 0;
    _rs = 0;  // Command mode
    _spi.write(cmd);
    _cs = 1;
    wait_us(1);
}

void miniDisplay::writeData(uint8_t data) {
    _cs = 0;
    _rs = 1;  // Data mode
    _spi.write(data);
    _cs = 1;
    wait_us(1);
}

void miniDisplay::clear() {
    memset(_vram, 0, sizeof(_vram));
}

void miniDisplay::swizzleBits(const uint8_t* input, uint8_t* top, uint8_t* bottom) {
    // Convert from "rows of pixels" to "columns of pixels" format
    uint64_t t = 0, b = 0;
    
    // Process top half
    for (int row = 0; row < 8; row++) {
        uint64_t spaced = (input[row] * 0x8040201008040201ULL) & 0x8080808080808080ULL;
        t |= spaced >> (7 - row);
    }
    
    // Process bottom half
    for (int row = 0; row < 8; row++) {
        uint64_t spaced = (input[row + 8] * 0x8040201008040201ULL) & 0x8080808080808080ULL;
        b |= spaced >> (7 - row);
    }
    
    // Extract bytes
    for (int i = 0; i < 8; i++) {
        top[i] = (t >> (56 - i * 8)) & 0xFF;
        bottom[i] = (b >> (56 - i * 8)) & 0xFF;
    }
}


void miniDisplay::setContrast(uint8_t contrast) {
    writeCommand(0x81);  // Set contrast command
    writeCommand(contrast & 0x3F);  // Contrast value (0-63)
}

void miniDisplay::printf(const char* format, ...) {
    char temp[256];
    va_list args;
    va_start(args, format);
    vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);
    
    puts(temp);
}

void miniDisplay::puts(const char* str) {
    while (*str) {
        putChar(*str++);
    }
    if (_dirty) {
        renderLines();
    }
}


void miniDisplay::putChar(char c) {
    if (c == '\n') {
        // Move to next line
        _currentLine = (_currentLine + 1) % BUFFER_LINES;
        _lineBuffer[_currentLine].clear();
        
        // Scroll display if needed
        if ((_currentLine + BUFFER_LINES - _topLine) % BUFFER_LINES >= LINES) {
            _topLine = (_topLine + 1) % BUFFER_LINES;
        }
        _dirty = true;
    }
    else if (c >= ' ') {
        // Add character to current line
        if (_lineBuffer[_currentLine].length() < CHARS_PER_LINE) {
            _lineBuffer[_currentLine] += c;
            _dirty = true;
        }
        else {
            // Auto wrap
            putChar('\n');
            putChar(c);
        }
    }
}

void miniDisplay::renderLines() {
    // Clear display buffer
    memset(_displayBuffer, 0, sizeof(_displayBuffer));
    
    // Render visible lines
    for (int line = 0; line < LINES; line++) {
        int bufferLine = (_topLine + line) % BUFFER_LINES;
        const std::string& text = _lineBuffer[bufferLine];
        
        for (size_t i = 0; i < text.length() && i < CHARS_PER_LINE; i++) {
            drawChar(i * CHAR_WIDTH, line * CHAR_HEIGHT, text[i]);
        }
    }
}

void miniDisplay::drawChar(int x, int y, char c) {
    const uint8_t* charData = _font[c - 32];  // Adjust index for font
    
    // Each character is 8x14 pixels
    for (int row = 0; row < 14; row++) {
        int page = (y + row) / 8;
        int bit = (y + row) % 8;
        uint8_t line = charData[row];
        
        for (int col = 0; col < 8; col++) {
            if (line & (0x80 >> col)) {
                _displayBuffer[page * 128 + x + col] |= (1 << bit);
            }
        }
    }
}

void miniDisplay::drawPixel(int x, int y) {
    // Calculate which page (0-7) and which bit (0-7) in that page
    int page = y / 8;        // Which group of 8 vertical pixels
    int bit = y % 8;         // Which bit in that group
    
    // Set the bit in the correct byte of the display buffer
    _displayBuffer[page * 128 + x] |= (1 << bit);
}

// Example: Draw a simple box
void miniDisplay::drawBox(int x1, int y1, int width, int height) {
    for (int x = x1; x < x1 + width; x++) {
        for (int y = y1; y < y1 + height; y++) {
            drawPixel(x, y);
        }
    }
}

// Example: Draw a single vertical line of the character
void miniDisplay::drawVerticalLine(int x, uint8_t data) {
    for (int bit = 0; bit < 8; bit++) {
        if (data & (1 << bit)) {
            drawPixel(x, bit);
        }
    }
}

// Example usage:
/*
int main() {
    miniDisplay display(
        SPI1_MOSI,  // MOSI pin
        SPI1_SCK,   // SCK pin
        LCD_RS,     // RS/DC pin
        LCD_ENA,    // CS pin
        RESET       // Reset pin
    );
    
    display.init();
    display.clear();
    
    while(1) {
        ThisThread::sleep_for(100ms);
    }
}
*/
