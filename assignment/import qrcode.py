import qrcode

# The data or URL you want to encode
data = 'ENROLMENT NO : 2253671437, PROGRAM : MHD, NAME : SATYAM PRAKASH, C/O PREM PRAKASH, ADMN. YEAR: 2024 - JULY, ADMN. VALID UPTO: 2027 - JUNE, MARKS : 460.15/800, PERCENTAGE : 57.52, DIVISION : SECOND DIVISION,'

# Configure the QR code
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)

qr.add_data(data)
qr.make(fit=True)

# Create and save the image
img = qr.make_image(fill_color="black", back_color="white")
img.save("qrcode.png")

print("QR code generated and saved as qrcode.png")