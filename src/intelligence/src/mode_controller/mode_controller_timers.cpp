#include "mode_controller/mode_controller.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <regex>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace boarai::intelligence
{

  namespace
  {
    auto limit_topic_matcher = std::regex{"^/boarai/.*/limit/.*$"};
  }

  auto mode_controller::start_timers() -> void
  {
    m_limit_subscriber_timer = create_wall_timer(2s, std::bind(&mode_controller::on_limit_subscriber_timer_expired, this));
  }

  auto mode_controller::on_limit_subscriber_timer_expired() -> void
  {
    auto topics = get_topic_names_and_types();
    auto topics_to_subscribe = std::vector<decltype(topics)::value_type>{};

    copy_if(cbegin(topics), cend(topics), back_inserter(topics_to_subscribe), [this](auto entry) {
      auto name = entry.first;
      return !m_limit_subscriptions.count(name) && std::regex_match(name, limit_topic_matcher);
    });

    for_each(cbegin(topics_to_subscribe), cend(topics_to_subscribe), [this](auto entry) {
      log_info("subscribing to {}", entry.first);
      subscribe_to_limit_topic(entry.first, entry.second.front());
    });
  }

}  // namespace boarai::intelligence
