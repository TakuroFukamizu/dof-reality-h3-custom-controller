#pragma once

// USB の1行を分解する。数値例は書式であり、推奨する運転値ではない。

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

enum class CmdKind : uint8_t {
  None,
  Help,
  Status,
  Arm,
  Disarm,
  PowerOn,
  PowerOff,
  BrakeRelease,
  BrakeApply,
  Run,
  Stop,
  Estop,
  FaultReset,
  ZeroDisplay,
  MarkStart,
  MarkEnd,
  VisualConfig,
  VisualReference,
  VisualFinish,
  VisualCancel,
  VisualResult,
  CalAngle,
  CalRevs,
  StreamOn,
  StreamOff,
  Heartbeat,
  Set,
  Config,
  AuthorityLocal,
  AuthorityUsb,
  Unknown
};

struct ParsedCommand {
  CmdKind kind;
  bool ok;
  const char* error;
  int duty_pct;
  int direction;
  int duration_ms;
  int revolutions;
  int max_drive_ms;
  int max_travel_counts;
  double number;
  char shaft[16];
  char reference[48];
  char key[32];
  char value[48];
};

inline bool cmd_eq(const char* left, const char* right) {
  if (left == nullptr || right == nullptr) {
    return false;
  }
  while (*left != '\0' && *right != '\0') {
    char a = *left;
    char b = *right;
    if (a >= 'A' && a <= 'Z') {
      a = static_cast<char>(a - 'A' + 'a');
    }
    if (b >= 'A' && b <= 'Z') {
      b = static_cast<char>(b - 'A' + 'a');
    }
    if (a != b) {
      return false;
    }
    ++left;
    ++right;
  }
  return *left == *right;
}

inline const char* cmd_skip(const char* text) {
  while (*text == ' ' || *text == '\t' || *text == '\r') {
    ++text;
  }
  return text;
}

inline bool cmd_token(const char** cursor, char* out, int size) {
  if (cursor == nullptr || *cursor == nullptr || out == nullptr || size <= 1) {
    return false;
  }
  *cursor = cmd_skip(*cursor);
  if (**cursor == '\0') {
    out[0] = '\0';
    return false;
  }
  int used = 0;
  while (**cursor != '\0' && **cursor != ' ' && **cursor != '\t') {
    if (used < size - 1) {
      out[used++] = **cursor;
    }
    ++(*cursor);
  }
  out[used] = '\0';
  return true;
}

inline void cmd_upper(char* text) {
  if (text == nullptr) {
    return;
  }
  for (; *text != '\0'; ++text) {
    if (*text >= 'a' && *text <= 'z') {
      *text = static_cast<char>(*text - 'a' + 'A');
    }
  }
}

inline bool cmd_key_here(const char* text, const char* key) {
  while (*key != '\0') {
    char have = *text;
    char want = *key;
    if (have >= 'A' && have <= 'Z') {
      have = static_cast<char>(have - 'A' + 'a');
    }
    if (want >= 'A' && want <= 'Z') {
      want = static_cast<char>(want - 'A' + 'a');
    }
    if (have != want) {
      return false;
    }
    ++text;
    ++key;
  }
  return *text == '=';
}

inline bool cmd_copy_value(const char* text, char* out, int size) {
  if (out == nullptr || size <= 1) {
    return false;
  }
  int used = 0;
  if (*text == '"') {
    ++text;
    while (*text != '\0' && *text != '"') {
      if (used < size - 1) {
        out[used++] = *text;
      }
      ++text;
    }
    out[used] = '\0';
    return used > 0 && *text == '"';
  }
  while (*text != '\0' && *text != ' ' && *text != '\t') {
    if (used < size - 1) {
      out[used++] = *text;
    }
    ++text;
  }
  out[used] = '\0';
  return used > 0;
}

inline bool cmd_find(const char* line, const char* key, char* out, int size) {
  const char* cursor = line;
  while (*cursor != '\0') {
    if ((cursor == line || cursor[-1] == ' ' || cursor[-1] == '\t') && cmd_key_here(cursor, key)) {
      const char* value = cursor + strlen(key) + 1;
      return cmd_copy_value(value, out, size);
    }
    ++cursor;
  }
  return false;
}

inline bool cmd_direction(const char* text, int* direction) {
  if (cmd_eq(text, "positive") || cmd_eq(text, "pos") || cmd_eq(text, "+1") || cmd_eq(text, "1") ||
      cmd_eq(text, "+")) {
    *direction = 1;
    return true;
  }
  if (cmd_eq(text, "negative") || cmd_eq(text, "neg") || cmd_eq(text, "-1") || cmd_eq(text, "-")) {
    *direction = -1;
    return true;
  }
  return false;
}

inline ParsedCommand cmd_reject(CmdKind kind, const char* error) {
  ParsedCommand command{};
  command.kind = kind;
  command.ok = false;
  command.error = error;
  return command;
}

inline ParsedCommand cmd_ok(CmdKind kind) {
  ParsedCommand command{};
  command.kind = kind;
  command.ok = true;
  command.error = "";
  return command;
}

inline ParsedCommand parse_command(const char* line) {
  if (line == nullptr) {
    return cmd_reject(CmdKind::Unknown, "empty");
  }
  char buf[192];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  const char* cursor = buf;
  char first[24];
  char second[24];
  if (!cmd_token(&cursor, first, sizeof(first))) {
    return cmd_reject(CmdKind::None, "empty");
  }
  const bool has_second = cmd_token(&cursor, second, sizeof(second));
  const char* rest = cmd_skip(cursor);

  if (cmd_eq(first, "help") && !has_second) return cmd_ok(CmdKind::Help);
  if (cmd_eq(first, "status") && !has_second) return cmd_ok(CmdKind::Status);
  if (cmd_eq(first, "arm") && !has_second) return cmd_ok(CmdKind::Arm);
  if (cmd_eq(first, "disarm") && !has_second) return cmd_ok(CmdKind::Disarm);
  if (cmd_eq(first, "stop") && !has_second) return cmd_ok(CmdKind::Stop);
  if (cmd_eq(first, "estop") && !has_second) return cmd_ok(CmdKind::Estop);
  if (cmd_eq(first, "heartbeat") && !has_second) return cmd_ok(CmdKind::Heartbeat);
  if (cmd_eq(first, "config") && !has_second) return cmd_ok(CmdKind::Config);

  if (cmd_eq(first, "power") && has_second && rest[0] == '\0') {
    if (cmd_eq(second, "on")) return cmd_ok(CmdKind::PowerOn);
    if (cmd_eq(second, "off")) return cmd_ok(CmdKind::PowerOff);
    return cmd_reject(CmdKind::Unknown, "power_arg");
  }
  if (cmd_eq(first, "brake") && has_second && rest[0] == '\0') {
    if (cmd_eq(second, "release")) return cmd_ok(CmdKind::BrakeRelease);
    if (cmd_eq(second, "apply")) return cmd_ok(CmdKind::BrakeApply);
    return cmd_reject(CmdKind::Unknown, "brake_arg");
  }
  if (cmd_eq(first, "fault") && has_second && cmd_eq(second, "reset") && rest[0] == '\0') {
    return cmd_ok(CmdKind::FaultReset);
  }
  if (cmd_eq(first, "zero") && has_second && cmd_eq(second, "display") && rest[0] == '\0') {
    return cmd_ok(CmdKind::ZeroDisplay);
  }
  if (cmd_eq(first, "mark") && has_second && rest[0] == '\0') {
    if (cmd_eq(second, "start")) return cmd_ok(CmdKind::MarkStart);
    if (cmd_eq(second, "end")) return cmd_ok(CmdKind::MarkEnd);
    return cmd_reject(CmdKind::Unknown, "mark_arg");
  }
  if (cmd_eq(first, "stream") && has_second && rest[0] == '\0') {
    if (cmd_eq(second, "on")) return cmd_ok(CmdKind::StreamOn);
    if (cmd_eq(second, "off")) return cmd_ok(CmdKind::StreamOff);
    return cmd_reject(CmdKind::Unknown, "stream_arg");
  }
  if (cmd_eq(first, "authority") && has_second && rest[0] == '\0') {
    if (cmd_eq(second, "local")) return cmd_ok(CmdKind::AuthorityLocal);
    if (cmd_eq(second, "usb")) return cmd_ok(CmdKind::AuthorityUsb);
    return cmd_reject(CmdKind::Unknown, "authority_arg");
  }
  if (cmd_eq(first, "cal") && has_second) {
    char number_text[32];
    const char* number_cursor = rest;
    if (!cmd_token(&number_cursor, number_text, sizeof(number_text)) || cmd_skip(number_cursor)[0] != '\0') {
      return cmd_reject(CmdKind::Unknown, "cal_arg");
    }
    char* end = nullptr;
    const double number = strtod(number_text, &end);
    if (end == number_text || *end != '\0') {
      return cmd_reject(CmdKind::Unknown, "cal_arg");
    }
    ParsedCommand command = cmd_ok(cmd_eq(second, "angle") ? CmdKind::CalAngle : CmdKind::CalRevs);
    if (!cmd_eq(second, "angle") && !cmd_eq(second, "revs")) {
      return cmd_reject(CmdKind::Unknown, "cal_arg");
    }
    command.kind = cmd_eq(second, "angle") ? CmdKind::CalAngle : CmdKind::CalRevs;
    command.number = number;
    return command;
  }
  if (cmd_eq(first, "set") && has_second) {
    char raw_value[48];
    const char* value_cursor = rest;
    if (!cmd_token(&value_cursor, raw_value, sizeof(raw_value)) || cmd_skip(value_cursor)[0] != '\0') {
      return cmd_reject(CmdKind::Set, "set_arg");
    }
    ParsedCommand command = cmd_ok(CmdKind::Set);
    strncpy(command.key, second, sizeof(command.key) - 1);
    cmd_upper(command.key);
    strncpy(command.value, raw_value, sizeof(command.value) - 1);
    return command;
  }
  if (cmd_eq(first, "run")) {
    const char* body = has_second ? second : "";
    char joined[160];
    joined[0] = '\0';
    if (has_second) {
      strncpy(joined, second, sizeof(joined) - 1);
      if (rest[0] != '\0') {
        const int used = static_cast<int>(strlen(joined));
        if (used + 1 < static_cast<int>(sizeof(joined))) {
          joined[used] = ' ';
          joined[used + 1] = '\0';
          strncat(joined, rest, sizeof(joined) - strlen(joined) - 1);
        }
      }
    }
    (void)body;
    char duty_text[16];
    char dir_text[16];
    char time_text[16];
    if (!cmd_find(joined, "duty_pct", duty_text, sizeof(duty_text)) ||
        !cmd_find(joined, "direction", dir_text, sizeof(dir_text)) ||
        !cmd_find(joined, "duration_ms", time_text, sizeof(time_text))) {
      return cmd_reject(CmdKind::Run, "run_arg");
    }
    int duty = 0;
    int duration = 0;
    char* end = nullptr;
    const long duty_long = strtol(duty_text, &end, 10);
    if (end == duty_text || *end != '\0') {
      return cmd_reject(CmdKind::Run, "run_arg");
    }
    duty = static_cast<int>(duty_long);
    end = nullptr;
    const long duration_long = strtol(time_text, &end, 10);
    if (end == time_text || *end != '\0') {
      return cmd_reject(CmdKind::Run, "run_arg");
    }
    duration = static_cast<int>(duration_long);
    int direction = 0;
    if (!cmd_direction(dir_text, &direction)) {
      return cmd_reject(CmdKind::Run, "run_arg");
    }
    ParsedCommand command = cmd_ok(CmdKind::Run);
    command.duty_pct = duty;
    command.direction = direction;
    command.duration_ms = duration;
    return command;
  }
  if (cmd_eq(first, "visual") && has_second) {
    if (cmd_eq(second, "reference") && rest[0] == '\0') return cmd_ok(CmdKind::VisualReference);
    if (cmd_eq(second, "finish") && rest[0] == '\0') return cmd_ok(CmdKind::VisualFinish);
    if (cmd_eq(second, "cancel") && rest[0] == '\0') return cmd_ok(CmdKind::VisualCancel);
    if (cmd_eq(second, "result") && rest[0] == '\0') return cmd_ok(CmdKind::VisualResult);
    if (!cmd_eq(second, "config")) {
      return cmd_reject(CmdKind::Unknown, "visual_arg");
    }
    char joined[180];
    strncpy(joined, rest, sizeof(joined) - 1);
    joined[sizeof(joined) - 1] = '\0';
    char shaft[16];
    char reference[48];
    char dir_text[16];
    char rev_text[16];
    char drive_text[16];
    char travel_text[16];
    if (!cmd_find(joined, "shaft", shaft, sizeof(shaft)) ||
        !cmd_find(joined, "reference", reference, sizeof(reference)) ||
        !cmd_find(joined, "direction", dir_text, sizeof(dir_text)) ||
        !cmd_find(joined, "revolutions", rev_text, sizeof(rev_text)) ||
        !cmd_find(joined, "max_drive_ms", drive_text, sizeof(drive_text)) ||
        !cmd_find(joined, "max_travel_counts", travel_text, sizeof(travel_text))) {
      return cmd_reject(CmdKind::VisualConfig, "visual_config");
    }
    int direction = 0;
    if (!cmd_direction(dir_text, &direction)) {
      return cmd_reject(CmdKind::VisualConfig, "visual_config");
    }
    char* end = nullptr;
    const long revolutions = strtol(rev_text, &end, 10);
    if (end == rev_text || *end != '\0') return cmd_reject(CmdKind::VisualConfig, "visual_config");
    end = nullptr;
    const long drive_ms = strtol(drive_text, &end, 10);
    if (end == drive_text || *end != '\0') return cmd_reject(CmdKind::VisualConfig, "visual_config");
    end = nullptr;
    const long travel = strtol(travel_text, &end, 10);
    if (end == travel_text || *end != '\0') return cmd_reject(CmdKind::VisualConfig, "visual_config");
    ParsedCommand command = cmd_ok(CmdKind::VisualConfig);
    strncpy(command.shaft, shaft, sizeof(command.shaft) - 1);
    strncpy(command.reference, reference, sizeof(command.reference) - 1);
    command.direction = direction;
    command.revolutions = static_cast<int>(revolutions);
    command.max_drive_ms = static_cast<int>(drive_ms);
    command.max_travel_counts = static_cast<int>(travel);
    return command;
  }
  return cmd_reject(CmdKind::Unknown, "unknown");
}
