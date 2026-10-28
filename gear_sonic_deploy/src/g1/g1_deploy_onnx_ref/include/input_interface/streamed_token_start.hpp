#ifndef STREAMED_TOKEN_START_HPP
#define STREAMED_TOKEN_START_HPP

namespace sonic {
enum class TokenStartResult { NoRequest, Stopped, AlreadyStarted, MissingToken, Applied };
// Called after pose input processing, so a stop from either input wins.
// A rejected request is not retained for a future frame.
inline TokenStartResult ApplyTokenStart(bool request, bool stop, bool token_ready,
                                       bool& started, bool& play, bool& reinitialize_heading) {
  if (!request) return TokenStartResult::NoRequest;
  if (stop) return TokenStartResult::Stopped;
  if (started) return TokenStartResult::AlreadyStarted;
  if (!token_ready) return TokenStartResult::MissingToken;
  started = true;
  play = false;
  reinitialize_heading = true;
  return TokenStartResult::Applied;
}
}
#endif
