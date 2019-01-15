
void dwGetSystemTimestamp(dwTime_t *time)
{
    // dwSpiRead(dev, SYS_TIME, NO_SUB, time->raw, LEN_SYS_TIME);
    dwt_readfromdevice(SYS_TIME_ID, SYS_TIME_OFFSET, SYS_TIME_LEN, time->raw);
}

void dwSetData(dwDevice_t *dev, uint8_t data[], unsigned int n)
{
    if (dev->frameCheck)
    {
        n += 2; // two bytes CRC-16
    }
    if (n > LEN_EXT_UWB_FRAMES)
    {
        return; // TODO proper error handling: frame/buffer size
    }
    if (n > LEN_UWB_FRAMES && !dev->extendedFrameLength)
    {
        return; // TODO proper error handling: frame/buffer size
    }
    // transmit data and length
    dwt_writetxdata(n, (uint8_t *)data, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(n, 0, 1);              /* Zero offset in TX buffer, no ranging. */
}

int8_t dwStartTransmit(dwTime_t *txTime)
{

    int ret;
    int8_t irq_status = dw1000_disable_interrupt();
    /* Switch off radio before setting it to transmit
     * It also clears pending interrupts */
    dwt_forcetrxoff();

    dwt_setdelayedtrxtime(txTime.low32);
    /* Radio starts listening certain delay (in UWB microseconds) after TX */
    dwt_setrxaftertxdelay(0);

    /* Start transmission, indicating that a response is expected so that reception
     * is enabled automatically after the frame is sent and the delay set by
     * dwt_setrxaftertxdelay() has elapsed. */
    ret = dwt_starttx(3);
    dw1000_enable_interrupt(irq_status);

    return ret;
}

void dwStartReceive(dwDevice_t *dev)
{
    setBit(dev->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dev->frameCheck);
    setBit(dev->sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
    dwSpiWrite(dev, SYS_CTRL, NO_SUB, dev->sysctrl, LEN_SYS_CTRL);
    dwt_readrxdata(buf, buf_len, 0);
}

void rx_rng_ok_cb(const dwt_cb_data_t *cb_data)
{
    uint16_t pkt_len = cb_data->datalength;
    // if(pkt_len != 12) {
    //     goto abort;
    // }
    dwt_readrxtimestamp(rxTime.raw);
    // abort: /* In case we got anything unexpected */
    //   dwt_forcetrxoff();
    //   dwt_rxreset(); /* just to check */
    //   dwt_setrxtimeout(0);
    //   dwt_rxenable(0);
}