#include "bsp.h"
#include "config.h"

enum ActionType {
  ACTION_FORWARD,
  ACTION_LEFT_TURN,
  ACTION_RIGHT_TURN,
  ACTION_BACK_TURN,
  ACTION_PUSH_BLOCKS,
  ACTION_WAIT
};

struct RouteStep {
  ActionType action;
  uint16_t value;
};

// Keep your existing route
static const RouteStep selectionRoute[] = {
  {ACTION_LEFT_TURN, 0},
  {ACTION_FORWARD, 1},
  {ACTION_WAIT, 200},

  {ACTION_LEFT_TURN, 0},
  {ACTION_FORWARD, 1},
  {ACTION_WAIT, 200},

  {ACTION_RIGHT_TURN, 0},
  {ACTION_FORWARD, 1},
  {ACTION_WAIT, 200},

  {ACTION_BACK_TURN, 0},
  {ACTION_FORWARD, 1},
  {ACTION_WAIT, 200},

  {ACTION_LEFT_TURN, 0},
  {ACTION_FORWARD, 2},
  {ACTION_RIGHT_TURN, 0},
  {ACTION_FORWARD, 1},

  {ACTION_PUSH_BLOCKS, 0}
};

static const uint8_t selectionRouteLength = sizeof(selectionRoute) / sizeof(selectionRoute[0]);

static void WaitForStartSignal() {
  pinMode(KEY_PIN, INPUT_PULLUP);

  while (digitalRead(KEY_PIN) == HIGH) delay(1);
  delay(START_BUTTON_DEBOUNCE_MS);
  while (digitalRead(KEY_PIN) == LOW) delay(1);
}

static void ExecuteStep(const RouteStep &step) {
  switch (step.action) {
    case ACTION_FORWARD:      Forward((uint8_t)step.value); break;
    case ACTION_LEFT_TURN:    LeftTurn(); break;
    case ACTION_RIGHT_TURN:   RightTurn(); break;
    case ACTION_BACK_TURN:    BackTurn(); break;
    case ACTION_PUSH_BLOCKS:  PushBlocks(); break;
    case ACTION_WAIT:         delay(step.value); break;
  }
}

void setup() {
  Serial.begin(9600);
  Motor_Init();
  Trackline_Init();
  delay(POWER_ON_SETTLE_MS);
  WaitForStartSignal();
}

void loop() {
  static bool done = false;
  if (done) { Stop(); return; }

  for (uint8_t i = 0; i < selectionRouteLength; ++i) {
    ExecuteStep(selectionRoute[i]);
  }
  Stop();
  done = true;
}
