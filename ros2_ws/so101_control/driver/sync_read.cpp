// sync_read.cpp
#include "SMS_STS.h"
#include <iostream>
#include <unistd.h>

SMS_STS sm_st;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " /dev/ttyACM0 <ID>\n";
    std::cerr << "Example: " << argv[0] << " /dev/ttyACM0 1\n";
    return 1;
  }

  const char *port = argv[1];
  int id = std::stoi(argv[2]);

  std::cout << "Opening " << port << " @ 1,000,000 baud, ID=" << id << "...\n";

  if (!sm_st.begin(1000000, port)) {
    std::cerr << "Failed to open serial port!\n";
    return 1;
  }
  usleep(20000);

  // Direct read
  int pos = sm_st.ReadPos(id);
  if (pos != -1) {
    std::cout << "SERVO DETECTED! Position=" << pos << "\n";
  } else {
    std::cerr << "NO RESPONSE from ReadPos()\n";
  }

  sm_st.EnableTorque(id, 1);
  std::cout << "Torque ON (ID=" << id << ")\n";

  // CORRECT: Use FeedBack() to detect servo
  if (sm_st.FeedBack(id) != -1) {
    std::cout << "SERVO DETECTED! ID=" << id << " (STS3215)\n";
  } else {
    std::cerr << "NO RESPONSE – check ID, power, or wiring\n";
    sm_st.end();
    return 1;
  }

  while (true) {
    int pos = sm_st.ReadPos(id);
    if (pos != -1) {
      std::cout << "ID=" << id << " Pos=" << pos << " (0-4095)\n";
    } else {
      std::cout << "read pos err\n";
    }
    usleep(100000);
  }

  sm_st.end();
  return 0;
}