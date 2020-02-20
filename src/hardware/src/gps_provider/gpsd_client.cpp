#include "gps_provider/gpsd_client.hpp"

#include "gps_provider/gpsmm_adapter.hpp"

#include <cassert>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

namespace boarai::hardware
{

  gpsd_client::gpsd_client(listener & listener, std::string const & host, std::uint16_t port)
      : m_context{host.c_str(), std::to_string(port).c_str()}
      , m_runner_handle{}
      , m_update_listener{listener}
      , m_do_update{}
  {
    if (!m_context.stream(WATCH_ENABLE | WATCH_JSON))
    {
      throw std::invalid_argument{"failed to connect to GPS daemon"};
    }
  }

  gpsd_client::~gpsd_client() noexcept
  {
    stop();
  }

  auto gpsd_client::start() -> void
  {
    assert(!m_runner_handle.valid());
    m_do_update = true;
    m_runner_handle = async(std::launch::async, std::bind(&gpsd_client::do_update, this));
  }

  auto gpsd_client::stop() -> void
  {
    m_do_update = false;
    if (m_runner_handle.valid())
    {
      m_runner_handle.get();
    }
  }

  auto gpsd_client::do_update() -> void
  {
    while (m_do_update)
    {
      if (!m_context.waiting(1'000))
      {
        continue;
      }

      auto raw_data = m_context.read();
      if (!raw_data)
      {
        m_do_update = false;
        continue;
      }

      try
      {
        m_update_listener.on_new_data(*raw_data);
      }
      catch (std::exception const &)
      {
      }
    }
  }

}  // namespace boarai::hardware