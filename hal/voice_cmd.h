#ifndef CROSSFIRE_VOICE_CMD_H
#define CROSSFIRE_VOICE_CMD_H

/*
 * 语音播报命令构造（纯逻辑，不依赖 system/OpenCV，可独立测试）。
 * 输入：播报文本 或 音频文件路径（.mp3/.wav）。
 * 输出：交给 system() 执行的 shell 命令。
 */

#include <string>

std::string build_voice_command(const std::string& text_or_path);

#endif /* CROSSFIRE_VOICE_CMD_H */
