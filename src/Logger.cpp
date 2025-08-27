#include <Arduino.h>
#include <stdlib.h>
#include <vector>
using namespace std;

class PIDLogger
{
    static const size_t MAX_ENTRIES = 500;
    static const uint32_t RATE_LIMIT_MS = 20;

    struct Entry
    {
        short line;
        short output;
        short scale;
        short left;
        short right;
    };

    Entry buffer[MAX_ENTRIES];
    size_t index = 0;
    bool wrapped = false;
    uint32_t lastLogTime = 0;

public:
    // Returns true if logged, false if rate-limited
    bool log(short line, short output, float scale, short left, short right)
    {
        uint32_t now = millis();
        if (now - lastLogTime < RATE_LIMIT_MS)
            return false;
        lastLogTime = now;

        buffer[index++] = {line, output, scale: (short)(scale * 100.0), left, right};
        if (index >= MAX_ENTRIES)
        {
            index = 0;
            wrapped = true;
        }
        return true;
    }

    void dump(Stream &out = Serial)
    {
        size_t start = wrapped ? index : 0;
        size_t count = wrapped ? MAX_ENTRIES : index;

        for (size_t i = 0; i < count; ++i)
        {
            size_t idx = (start + i) % MAX_ENTRIES;
            const Entry &e = buffer[idx];

            out.printf(">line:%d\n>output:%d\n>scale:%d\n>left:%d\n>right:%d\n", e.line, e.output, e.scale, e.left, e.right);
            delay(20);
        }
    }

    void clear()
    {
        index = 0;
        wrapped = false;
        lastLogTime = 0;
    }
};

class SensorStripLogger
{
    size_t MAX_ENTRIES = 20;
    size_t SENSOR_COUNT = 8;
    vector<vector<short>> buffer;

    size_t index = 0;
    bool wrapped = false;

public:
    SensorStripLogger(size_t sensors, size_t entries = 20): MAX_ENTRIES(entries), SENSOR_COUNT(sensors) {
        buffer.resize(MAX_ENTRIES, vector<short>(SENSOR_COUNT));
    }

    // Log all sensor values at once
    bool log(const short sensorVals[])
    {
        for (size_t i = 0; i < SENSOR_COUNT; ++i)
            buffer[index][i] = sensorVals[i];
        index++;
        if (index >= MAX_ENTRIES)
        {
            index = 0;
            wrapped = true;
        }
        return true;
    }

    void dump(Stream &out = Serial)
    {
        // Print header
        for (size_t j = 0; j < SENSOR_COUNT; ++j) {
            out.print("A");
            out.print(j);
            if (j < SENSOR_COUNT - 1) out.print("\t");
        }
        out.println();

        size_t start = wrapped ? index : 0;
        size_t count = wrapped ? MAX_ENTRIES : index;

        for (size_t i = 0; i < count; ++i)
        {
            size_t idx = (start + i) % MAX_ENTRIES;
            for (size_t j = 0; j < SENSOR_COUNT; ++j)
            {
                out.print(buffer[idx][j]);
                if (j < SENSOR_COUNT - 1) out.print("\t");
            }
            out.println();
            delay(20);
        }
    }

    void clear()
    {
        index = 0;
        wrapped = false;
    }
};