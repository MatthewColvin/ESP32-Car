#pragma once
#include "driver/rmt_types.h"

#include <tuple>
#include <optional>

class IrNECParser
{
public:
    IrNECParser(){};
    struct Data
    {
        Data(uint16_t aAddr, uint16_t aData, bool aIsRepeat)
        {
            address = aAddr;
            data = aData;
            isRepeat = aIsRepeat;
        };
        uint16_t address = 0;
        uint16_t data = 0;
        bool isRepeat = false;
    };

    /**
     * @brief Use Parser to parse Remote Symbols and return some data
     *        if it is a valid NEC transmission
     *
     * @param aDoneEvent - Event From RMT library after calling rmt_receive()
     * @return std::optional<IrNECParser::Data>
     *         std::nullopt - Parse was unsuccessful
     *         IrNECParser::Data packed with address and data.
     */
    [[nodiscard]] std::optional<IrNECParser::Data> Parse(rmt_rx_done_event_data_t aDoneEvent);

private:
    static constexpr auto IR_NEC_DECODE_MARGIN = 200; // Tolerance for parsing RMT symbols into bit stream
    /**
     * @brief NEC timing spec
     */
    static constexpr auto NEC_LEADING_CODE_DURATION_0 = 9000;
    static constexpr auto NEC_LEADING_CODE_DURATION_1 = 4500;
    static constexpr auto NEC_PAYLOAD_ZERO_DURATION_0 = 560;
    static constexpr auto NEC_PAYLOAD_ZERO_DURATION_1 = 560;
    static constexpr auto NEC_PAYLOAD_ONE_DURATION_0 = 560;
    static constexpr auto NEC_PAYLOAD_ONE_DURATION_1 = 1690;
    static constexpr auto NEC_REPEAT_CODE_DURATION_0 = 9000;
    static constexpr auto NEC_REPEAT_CODE_DURATION_1 = 2250;
    // Functions from EspIDF example to parse NEC codes
    bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration);
    bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols);
    bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols);
    bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols);
    bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols);
    bool example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num);

    uint16_t mNecCodeAddress;
    uint16_t mNecCodeCommand;
    bool mIsRepeat = false;
};