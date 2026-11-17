#include "voice_cmd.h"

namespace {
std::string shell_quote(const std::string& value) {
    std::string out = "'";
    for (char c : value) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}
}

// Compatibility helper for older launch scripts. The production RealVoice implementation
// uses fork/exec with argv (no shell) and does not execute this returned string.
std::string build_voice_command(const std::string& text) {
    const bool is_file = text.size() >= 4 &&
        (text.rfind(".mp3") == text.size() - 4 || text.rfind(".wav") == text.size() - 4 ||
         text.rfind(".MP3") == text.size() - 4 || text.rfind(".WAV") == text.size() - 4);
    const std::string arg = shell_quote(text);
    if (is_file) {
        return "if command -v mpg123 >/dev/null 2>&1; then mpg123 -q -f 16384 " + arg + "; "
               "elif command -v ffplay >/dev/null 2>&1; then ffplay -nodisp -autoexit -volume 50 -loglevel quiet " + arg + "; "
               "elif command -v cvlc >/dev/null 2>&1; then cvlc --play-and-exit --gain 0.5 --intf dummy " + arg + "; "
               "else echo '[voice] no player'; fi";
    }
    return "if command -v espeak >/dev/null 2>&1; then espeak -v zh -s 160 " + arg + "; "
           "else echo '[voice] no TTS engine'; fi";
}
