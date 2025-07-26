# usbcanfd

A simple usbcanfd firmware implement for stm32g431 platform.

canable2是市面上非常廉价的usbcanfd工具，但唯一支持canfd的slcan固件不能做到差强人意，故此重新做一套软件。

使用stm32cubeide开发，直接生成usb-cdc的软件代码。使用usb-cdc串口协议，或许不能有效利用usb带宽，但不再需要usb驱动，使用python利用serial库开发pc端的工具也非常容易。

主机和设备之间的交互协议如下：

1. tfp: tiny frame protocol

串口数据流中的帧格式定义为：
```
head(4B) length(2B) payload(*B) crc(2B) tail(4B)
```

其中，`head`值为`0x68656164`（即`'head'`），`tail`值为`0x7461696cu`（即`'tail'`），`length`为`payload`的长度，`crc`为`payload`的checksum，目前暂未使用。除原始字节流外，所有数据均为大端序。

`tfp`协议中，每一帧存在12bytes的额外开销。

2. hsl: host slave link

消息格式定义为：
```
proto(1B) + mtype(1B) + stype(1B) + data(*B)
```

其中
- `proto`标识协议类型。可用来实现usb转can、usb转uart、usb转spi、usb转i2c等。CAN对应的`proto`字段固定为`1`。
- `mtype`标识消息类型。对于CAN，目前有`config(1)`、`connect(2)`、`unconnect(3)`、`transfer(4)`四种消息类型。对应的ACK消息类型最高位为`1`。
- `stype`标识消息子类型。目前仅在ACK消息中使用，用作错误码。
- `data`的内容和格式取决于`proto+mtype+stype`。

主从间的交互方式：
- 配置：
  - 主机发送config消息，数据长度10字节。依次为：仲裁域波特率（4字节）、仲裁域采样率（1字节）、数据域波特率（4字节）、仲裁域采样率（1字节）
  - 设备返回config响应，无数据。若`stype`为0，表示配置成功
- 连接：
  - 主机发送connect消息，无数据。
  - 设备返回connect响应，无数据。若`stype`为0，表示连接成功
- 断开：
  - 主机发送unconnect消息，无数据。
  - 设备返回unconnect响应，无数据。若`stype`为0，表示断开成功
- 收发：
  - 正常情况下，收发均无ACK。若发送失败，设备将返回`stype`非0的transfer响应

正常情况下，每一条transfer消息中包含一条CAN报文，其data字段的长度可从5字节到69字节，格式为：
```
frm_id(4B) + frm_type(1B) + frm_data(0B-64B)
```

其中，`frm_type`的bit0表示是否为扩展帧，bit1表示是否为fd帧，bit2表示是否启用brs。

