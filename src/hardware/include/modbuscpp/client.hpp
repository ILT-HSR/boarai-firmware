#ifndef BOARAI_HARDWARE_MODBUSCPP_CLIENT_HPP
#define BOARAI_HARDWARE_MODBUSCPP_CLIENT_HPP

#include "modbuscpp/address.hpp"
#include "modbuscpp/connection.hpp"
#include "modbuscpp/context.hpp"
#include "modbuscpp/error.hpp"

#include <modbus/modbus.h>

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <variant>
#include <vector>

namespace modbus
{
  namespace detail
  {
    struct single_value_datum_tag;
    struct multi_value_datum_tag;

    template<typename DatumType, typename CAPIReadType, typename DatumValueTag>
    struct readable_datum
    {
      auto static constexpr is_single_value = std::is_same_v<DatumValueTag, single_value_datum_tag>;

      using datum_type = std::conditional_t<is_single_value, DatumType, std::vector<DatumType>>;
      using getter_type = int (&)(modbus_t *, int, int, CAPIReadType *);

      readable_datum(getter_type getter,
                     context const & context,
                     address address,
                     std::uint16_t count,
                     modbus_error mdata_error_code)
          : m_getter{getter}
          , m_context{context}
          , m_address{address}
          , m_count{count}
          , m_mdata_error_code{mdata_error_code}
      {
      }

      auto operator*() const noexcept -> std::variant<datum_type, std::error_code>
      {
        auto api_handle = m_context.handle().get();
        auto api_address = static_cast<int>(m_address);

        auto api_value = [&] {
          if constexpr (is_single_value)
          {
            return CAPIReadType{};
          }
          else
          {
            return std::vector<CAPIReadType>(m_count);
          }
        }();

        auto const api_return_code = [&] {
          if constexpr (is_single_value)
          {
            return m_getter(api_handle, api_address, m_count, &api_value);
          }
          else
          {
            return m_getter(api_handle, api_address, m_count, api_value.data());
          }
        }();

        if (api_return_code < 0)
        {
          switch (errno)
          {
          case EMBMDATA:
            return make_error_code(m_mdata_error_code);
          default:
            return make_error_code(static_cast<std::errc>(errno));
          }
        }

        if constexpr (is_single_value)
        {
          return static_cast<datum_type>(api_value);
        }
        else
        {
          datum_type converted_value(m_count);
          transform(cbegin(api_value), cend(api_value), begin(converted_value), [](auto value) {
            return static_cast<typename datum_type::value_type>(value);
          });
          return converted_value;
        }
      }

      explicit operator datum_type() const
      {
        return std::get<datum_type>(**this);
      }

    protected:
      getter_type m_getter;
      context const & m_context;
      address m_address;
      std::uint16_t m_count;
      modbus_error m_mdata_error_code;
    };

    template<typename DatumType, typename CAPIReadType, typename CAPIWriteType, typename DatumValueTag>
    struct writeable_datum : readable_datum<DatumType, CAPIReadType, DatumValueTag>
    {
      using base_proxy = readable_datum<DatumType, CAPIReadType, DatumValueTag>;
      using datum_type = typename base_proxy::datum_type;
      using getter_type = typename base_proxy::getter_type;
      using setter_type = std::conditional_t<base_proxy::is_single_value,
                                             int (&)(modbus_t *, int, CAPIWriteType),
                                             int (&)(modbus_t *, int, int, CAPIWriteType const *)>;

      writeable_datum(getter_type getter,
                      setter_type setter,
                      context const & context,
                      address address,
                      std::uint16_t count,
                      modbus_error mdata_error_code)
          : base_proxy{getter, context, address, count, mdata_error_code}
          , m_setter{setter}
      {
      }

      auto operator=(datum_type value) noexcept -> std::error_code
      {
        auto api_handle = this->m_context.handle().get();
        auto api_address = static_cast<int>(this->m_address);
        auto api_return_code = [&] {
          if constexpr (base_proxy::is_single_value)
          {
            return m_setter(api_handle, api_address, static_cast<CAPIWriteType>(value));
          }
          else
          {
            auto api_value = std::vector<CAPIWriteType>(this->m_count);
            transform(cbegin(value), cend(value), begin(api_value), [](auto entry) {
              return static_cast<CAPIWriteType>(entry);
            });
            return m_setter(api_handle, api_address, this->m_count, api_value.data());
          }
        }();

        if (api_return_code < 0)
        {
          switch (errno)
          {
          case EMBMDATA:
            return make_error_code(this->m_mdata_error_code);
          default:
            return make_error_code(static_cast<std::errc>(errno));
          }
        }

        return {};
      }

    private:
      setter_type m_setter;
    };
  }  // namespace detail

  struct client
  {
    using sbit_readable_datum = detail::readable_datum<bool, std::uint8_t, detail::single_value_datum_tag>;
    using sbit_writeable_datum = detail::writeable_datum<bool, std::uint8_t, int, detail::single_value_datum_tag>;

    using mbit_readable_datum = detail::readable_datum<bool, std::uint8_t, detail::multi_value_datum_tag>;
    using mbit_writeable_datum = detail::writeable_datum<bool, std::uint8_t, std::uint8_t, detail::multi_value_datum_tag>;

    using sword_readable_datum = detail::readable_datum<std::bitset<16>, std::uint16_t, detail::single_value_datum_tag>;
    using sword_writeable_datum =
        detail::writeable_datum<std::bitset<16>, std::uint16_t, std::uint16_t, detail::single_value_datum_tag>;

    using mword_readable_datum = detail::readable_datum<std::bitset<16>, std::uint16_t, detail::multi_value_datum_tag>;
    using mword_writeable_datum =
        detail::writeable_datum<std::bitset<16>, std::uint16_t, std::uint16_t, detail::multi_value_datum_tag>;

    explicit client(connection const & connection);

    /**
     * Get an interface to a specific coil
     *
     * @param address The address of the coil
     */
    auto coil(address address) const noexcept -> sbit_writeable_datum;

    /**
     * Get an interface to a set of consecutive coils
     *
     * @param address The address of the first coil
     * @param count The number of coils, including the first coil
     */
    auto coils(address address, std::uint16_t count) const noexcept -> mbit_writeable_datum;

    /**
     * Get an interface to a specific discrete input
     *
     * @param address The address of the discrete input
     */
    auto discrete_input(address address) const noexcept -> sbit_readable_datum;

    /**
     * Get an interface to a set of consecutive discrete inputs
     *
     * @param address The address of the first discrete input
     * @param count The number of discrete inputs, including the first discrete input
     */
    auto discrete_inputs(address address, std::uint16_t count) const noexcept -> mbit_readable_datum;

    /**
     * Get an interface to a specific holding register
     *
     * @param address The address of the holding register
     */
    auto holding_register(address address) const noexcept -> sword_writeable_datum;

    /**
     * Get an interface to a set of consecutive holding registers
     *
     * @param address The address of the first holding register
     * @param count The number of holding registers, including the first holding register
     */
    auto holding_registers(address address, std::uint16_t count) const noexcept -> mword_writeable_datum;

    /**
     * Get an interface to a specific input register
     *
     * @param address The address of the input register
     */
    auto input_register(address address) const noexcept -> sword_readable_datum;

    /**
     * Get an interface to a set of consecutive input registers
     *
     * @param address The address of the first input register
     * @param count The number of input registers, including the first input register
     */
    auto input_registers(address address, std::uint16_t count) const noexcept -> mword_readable_datum;

  private:
    context const & m_context;
  };
}  // namespace modbus

#endif