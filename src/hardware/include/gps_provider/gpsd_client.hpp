#ifndef BOARAI_HARDWARE_GPSD_CLIENT_HPP
#define BOARAI_HARDWARE_GPSD_CLIENT_HPP

#include "gps_provider/gpsmm_adapter.hpp"

#include <libgpsmm.h>

#include <atomic>
#include <cstdint>
#include <future>
#include <string>

namespace boarai::hardware
{

  struct gpsd_client
  {
    struct listener
    {
      auto virtual on_new_data(gps_data_t data) -> void = 0;
    };

    explicit gpsd_client(listener & listener, std::string const & host, std::uint16_t port);

    ~gpsd_client() noexcept;

    auto start() -> void;
    auto stop() -> void;

  private:
    auto do_update() -> void;

    gpsmm m_context;
    std::future<void> m_runner_handle;
    listener & m_update_listener;
    std::atomic_bool m_do_update;
  };

}  // namespace boarai::hardware

#endif