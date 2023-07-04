#include "IrNECParser.hpp"

std::optional<IrNECParser::Data> IrNECParser::Parse(rmt_rx_done_event_data_t aDoneEvent)
{
    if (example_parse_nec_frame(aDoneEvent.received_symbols, aDoneEvent.num_symbols))
    {
        auto result = IrNECParser::Data(mNecCodeAddress, mNecCodeCommand, mIsRepeat);
        return std::make_optional(result);
    }
    return std::nullopt;
}

/**
 * @brief Saving NEC decode results
 */

bool IrNECParser::nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + IrNECParser::IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IrNECParser::IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
bool IrNECParser::nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, IrNECParser::NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, IrNECParser::NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
bool IrNECParser::nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, IrNECParser::NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, IrNECParser::NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
bool IrNECParser::nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, IrNECParser::NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, IrNECParser::NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code)
    {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            address |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            address &= ~(1 << i);
        }
        else
        {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            command |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            command &= ~(1 << i);
        }
        else
        {
            return false;
        }
        cur++;
    }
    // save address and command
    mNecCodeAddress = address;
    mNecCodeCommand = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
bool IrNECParser::nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, IrNECParser::NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, IrNECParser::NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
bool IrNECParser::example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    // printf("NEC frame start---\r\n");
    // for (size_t i = 0; i < symbol_num; i++)
    // {
    //     printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
    //            rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    // }
    // printf("---NEC frame end: ");
    switch (symbol_num)
    {
    case 34: // NEC normal frame
        return nec_parse_frame(rmt_nec_symbols);
        break;
    case 2: // NEC repeat frame
        return nec_parse_frame_repeat(rmt_nec_symbols);
        break;
    default:
        // printf("Unknown NEC frame\r\n\r\n");
        return false;
        break;
    }
}