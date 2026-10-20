import qrcode

qrcode.make("http://192.168.1.200").save("sigma_qr1.png")

print("Done — sigma_qr.png saved")
