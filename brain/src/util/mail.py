import sys
import os
import re

import smtplib 
import time

from email.MIMEText import MIMEText
from email.MIMEMultipart import MIMEMultipart
from email.mime.image import MIMEImage

alarm_type_map = {
        "UPGD1":"Unresponsive Person on Ground Detected",
        "RPGD1":"Responsive Person on Ground Detected",
        "UNKNOWN":"Uknown Problem Detected",
        }

class Mail(object):

    def __init__(self, host, port, username, password, encryption = True):
        self.__host = host
        self.__port = port
        self.__username = username
        self.__password = password
        self.__encryption = encryption

    def connect(self):
        self.__conn = smtplib.SMTP(host = self.__host, port = self.__port)
        if self.__encryption:
            self.__conn.ehlo()
            self.__conn.starttls()
            self.__conn.ehlo()
        self.__conn.set_debuglevel(False)
        self.__conn.login(self.__username, self.__password)


    def send_alarm(self, destination = "ronsnijdersron@gmail.com", location = "Unknown", client = "Person X", alarm_type = None, last_medication = "Unknown", use_image = False):
        self.connect()
        
        smoke = ""
        try:
            fp = open('/home/borg/smoke.png', 'rb')
            img = MIMEImage(fp.read())
            fp.close()
        except:
            smoke = "Smoke detected in the kitchen!"

        if not alarm_type:
            alarm_type = "UNKNOWN"

        datetime = time.strftime("%Y-%m-%d %H:%M:%S")
        content = """\
*** EMERGENCY ALARM (%s) ***
Client: %s
Type: %s: %s
Location: %s
Date/Time: %s
Last Medication: %s

%s

---
This is an automated email message, please do not reply.
"""\
            % (alarm_type, client, alarm_type, alarm_type_map[alarm_type], location, datetime, last_medication, smoke)

        msg = MIMEMultipart()

        msg['Subject'] = "EMERGENCY ALARM (%s)" % (alarm_type)
        msg['From'] = self.__username
        msg['To'] = destination

        text = MIMEText(content, 'plain')
        msg.attach(text)

        if use_image:
            try:
                fp = open('/home/borg/smoke.png', 'rb')
                img = MIMEImage(fp.read())
                fp.close()
                msg.attach(img)
            except:
                print "Unable to attach image."

        try:
            self.__conn.sendmail(self.__username, destination, msg.as_string())
        finally:
            self.__conn.close()


if __name__ == "__main__":
    mail = Mail(host = "mail.assistobot.com", port = 587, username = "borg", password = "happyhacking_333")
    mail.send_alarm(destination = "ronsnijdersron@gmail.com", location = "kitchen", client = "NL23423429423", alarm_type = "UPGD1", last_medication = "Pain Killers", use_image = False)
