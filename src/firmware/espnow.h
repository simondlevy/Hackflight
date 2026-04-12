/*
   ESPNOW message support

   Copyright (C) 2026 Simon D. Levy

   Based on: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files.  The above copyright
   notice and this permission notice shall be included in all copies or
   substantial portions of the Software.
*/


typedef struct {
  char a[32];
  int b;
  float c;
  bool d;
} espnow_message_t;
