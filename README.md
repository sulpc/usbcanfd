# usbcanfd

A simple usbcanfd firmware implement for stm32g431 platform.

canable2�������Ϸǳ����۵�usbcanfd���ߣ���Ψһ֧��canfd��slcan�̼�����������ǿ���⣬�ʴ�������һ�������

ʹ��stm32cubeide������ֱ������usb-cdc��������롣ʹ��usb-cdc����Э�飬��������Ч����usb������������Ҫusb������ʹ��python����serial�⿪��pc�˵Ĺ���Ҳ�ǳ����ס�

�������豸֮��Ľ���Э�����£�

1. tfp: tiny frame protocol

�����������е�֡��ʽ����Ϊ��
```
head(4B) length(2B) payload(*B) crc(2B) tail(4B)
```

���У�`head`ֵΪ`0x68656164`����`'head'`����`tail`ֵΪ`0x7461696cu`����`'tail'`����`length`Ϊ`payload`�ĳ��ȣ�`crc`Ϊ`payload`��checksum��Ŀǰ��δʹ�á���ԭʼ�ֽ����⣬�������ݾ�Ϊ�����

`tfp`Э���У�ÿһ֡����12bytes�Ķ��⿪����

2. hsl: host slave link

��Ϣ��ʽ����Ϊ��
```
proto(1B) + mtype(1B) + stype(1B) + data(*B)
```

����
- `proto`��ʶЭ�����͡�������ʵ��usbתcan��usbתuart��usbתspi��usbתi2c�ȡ�CAN��Ӧ��`proto`�ֶι̶�Ϊ`1`��
- `mtype`��ʶ��Ϣ���͡�����CAN��Ŀǰ��`config(1)`��`connect(2)`��`unconnect(3)`��`transfer(4)`������Ϣ���͡���Ӧ��ACK��Ϣ�������λΪ`1`��
- `stype`��ʶ��Ϣ�����͡�Ŀǰ����ACK��Ϣ��ʹ�ã����������롣
- `data`�����ݺ͸�ʽȡ����`proto+mtype+stype`��

���Ӽ�Ľ�����ʽ��
- ���ã�
  - ��������config��Ϣ�����ݳ���10�ֽڡ�����Ϊ���ٲ������ʣ�4�ֽڣ����ٲ�������ʣ�1�ֽڣ������������ʣ�4�ֽڣ����ٲ�������ʣ�1�ֽڣ�
  - �豸����config��Ӧ�������ݡ���`stype`Ϊ0����ʾ���óɹ�
- ���ӣ�
  - ��������connect��Ϣ�������ݡ�
  - �豸����connect��Ӧ�������ݡ���`stype`Ϊ0����ʾ���ӳɹ�
- �Ͽ���
  - ��������unconnect��Ϣ�������ݡ�
  - �豸����unconnect��Ӧ�������ݡ���`stype`Ϊ0����ʾ�Ͽ��ɹ�
- �շ���
  - ��������£��շ�����ACK��������ʧ�ܣ��豸������`stype`��0��transfer��Ӧ

��������£�ÿһ��transfer��Ϣ�а���һ��CAN���ģ���data�ֶεĳ��ȿɴ�5�ֽڵ�69�ֽڣ���ʽΪ��
```
frm_id(4B) + frm_type(1B) + frm_data(0B-64B)
```

���У�`frm_type`��bit0��ʾ�Ƿ�Ϊ��չ֡��bit1��ʾ�Ƿ�Ϊfd֡��bit2��ʾ�Ƿ�����brs��

