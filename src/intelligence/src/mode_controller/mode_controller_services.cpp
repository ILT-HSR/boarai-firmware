#include "mode_controller/mode_controller.hpp"
#include "support/interfaces.hpp"
#include "support/to_string.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::intelligence
{

  auto mode_controller::start_services() -> void
  {
    m_set_mode_service =
        create_service<service::set_mode_t>(service::set_mode, std::bind(&mode_controller::on_set_mode_request, this, _1, _2));
  }

  auto mode_controller::on_set_mode_request(service::set_mode_t::Request::SharedPtr request,
                                            service::set_mode_t::Response::SharedPtr response) -> void
  {
    auto raw_mode = request->mode;
    if (!is_valid<mode>(raw_mode))
    {
      response->accepted = false;
      return;
    }

    auto real_mode = from_string<mode>(raw_mode);
    if (!is_available(real_mode))
    {
      response->accepted = false;
      return;
    }

    change_to(real_mode);

    response->accepted = true;
  }

}  // namespace boarai::intelligence