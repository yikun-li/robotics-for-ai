
import brain
import logging
import util.nullhandler

import time

import random
import subprocess
import os
import os.path
import shutil

import cv

import numpy

logging_namespace = 'Borg.Brain.Util.Report'
logging.getLogger(logging_namespace).addHandler(util.nullhandler.NullHandler())

latex_header = """
\\documentclass[12pt]{article}
\\title{Emergency Situation: Report}
\\author{The BORG Team}
\\date{\\today}
\\usepackage{graphicx}
\\begin{document}
\\maketitle
"""

latex_footer = "\\end{document}"


def convert_topic(topic):
    topic_dict = {'hum':'Humidity (\\%)', 'temp':'Temperature ($^0$C)'}
    if topic in topic_dict:
        return topic_dict[topic]
    return topic


def latex_create_section(section, content):
    latex = "\\section*{%s}\n" % section
    latex += "\\begin{enumerate}\n"
    for (item_name, value) in content:
        latex += "    \\item[\\textbf{%s}]: %s\n" % (str(item_name), str(value))
    latex += "\\end{enumerate}\n"
    return latex


def value_list_to_list(value_list):
    the_list = []
    for value in value_list:
        the_list.append(value["value"])
    return the_list


class Report(object):
    """
    Used to store values of a given topic and create statistics about them.
    Able to produce report required for the emergency test.
    """

    def __init__(self, location):
        self.__topic_dict = {}
        self.__start_time = None
        self.__end_time = None
        self.__last_report = None
        self.__last_latex = None

        the_time = time.strftime("%Y-%m-%d_%H:%M")
        self.work = os.path.join(os.path.expanduser("~/EMERGENCY_WORK/%s" % the_time))
        self.location = location
        if os.path.exists(self.work):
            subprocess.Popen(["/bin/rm", "-r", self.work]).wait()
        os.makedirs(self.work)
        if not os.path.exists(location):
            os.makedirs(location)

        self.start()

    def start(self):
        self.__start_time = time.time()

    def stop(self):
        self.__end_time = time.time()

    def get_duration(self):
        end_time = time.time()
        if self.__end_time:
            end_time = self.__end_time
        return end_time - self.__start_time

    def add(self, topic, value, val_time = None, position = None):
        if val_time == None:
            val_time = time.time()
        if not topic in self.__topic_dict:
            self.__topic_dict[topic] = []
        self.__topic_dict[topic].append({"value":value, "time":val_time, "position":position})

    def get_sensor_info(self):
        sensor_info = {}
        for topic, value_list in self.__topic_dict.iteritems():
            v_list = value_list_to_list(value_list)
            if len(v_list) > 0 and (isinstance(v_list[0], int) or isinstance(v_list[0], float)):
                sensor_info[topic] = {}
                for func in ["mean", "std", "max", "min", "median"]:
                    exec("val = numpy.%s(%s)" % (func, v_list))
                    sensor_info[topic][func] = round(val, 2)
        return sensor_info

    def produce_latex(self):
        latex = ""
        latex += latex_header

        latex += self.latex_summary()
        latex += self.latex_persons()
        #latex += self.latex_sensor_values()

        latex += latex_footer

        self.__last_latex = latex

        return latex

    def latex_summary(self):
        path = os.path.join(self.work, "arena_result.png")
        latex = "\\section{Summary}\n"
        latex += "\\begin{enumerate}\n"
        latex += "    \\item[\\textbf{Start Time}]: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.__start_time))
        latex += "    \\item[\\textbf{Duration:}]: %.2f minutes\n" % round(self.get_duration() / 60, 2)
        latex += "\\end{enumerate}\n"
        latex += "\\includegraphics[height=8cm]{%s} \\ \n" % path
        latex += "\\begin{enumerate}\n"
        latex += "    \\item[\\textbf{Red}]: Persons that are NOT OK."
        latex += "    \\item[\\textbf{Orange}]: Persons that are OK and were guided to the exit."
        latex += "    \\item[\\textbf{Pink}]: Persons that are OK and went to the exit themselves."
        latex += "\\end{enumerate}\n"
        return latex

    def latex_sensor_values(self):
        sensor_info_dict = self.get_sensor_info()
        latex = "\\section{Sensor Values}\n"
        latex += "This section describes the sensor values measured during the search.\n"
        for topic, sensor_info in sensor_info_dict.iteritems():
            latex += "\\subsection{%s}\n" % convert_topic(topic)
            latex += "\\begin{enumerate}\n"
            for func, func_val in sensor_info.iteritems():
                latex += "    \\item[\\textbf{%s}]: %s\n" % (func, str(func_val))
            latex += "\\end{enumerate}\n"
        return latex

    def latex_persons(self):
        path = os.environ.get('BORG') + "/brain/src/util/arena.png"
        print "loading %s" % path
        im = cv.LoadImage(path)
        latex = "\\section{Persons}\n"
        latex += "This section describes the persons found in this building.\n"
        if 'person' in self.__topic_dict:
            no_person_not_ok = 0
            no_person_ok_exit = 0
            no_person_ok_guide = 0
            for person in self.__topic_dict['person']:
                if person['value'] == 'NOT_OK':
                    no_person_not_ok += 1
                elif person['value'] == 'OK_EXIT':
                    no_person_ok_exit += 1
                elif person['value'] == 'OK_GUIDE':
                    no_person_ok_guide += 1
            no_persons = no_person_not_ok + no_person_ok_exit + no_person_ok_guide
            latex += "\\begin{enumerate}\n"
            latex += "    \\item[\\textbf{Total amount:}] %d\n" % no_persons
            latex += "    \\item[\\textbf{NOT OK:}]\\footnote{Amount of persons found that were not OK.} %d\n" % no_person_not_ok
            latex += "    \\item[\\textbf{GUIDED:}]\\footnote{Amount of persons found that were OK and were guided to the exit.} %d\n" % no_person_ok_guide
            latex += "    \\item[\\textbf{OK:}]\\footnote{Amount of persons found that were OK and went to the exit themselves.} %d\n" % no_person_ok_exit
            latex += "\\end{enumerate}\n"

            
            offset = (83, 127)


            latex += "\\subsection{Persons: NOT OK}\n"
            latex += "Marked as \\textbf{red} in the map.\n"
            latex += "\\begin{enumerate}\n"
            count = 0
            x_scale = 20
            y_scale = -20
            for person in self.__topic_dict['person']:
                if person['value'] == 'NOT_OK':
                    count += 1
                    latex += "    \\item Location: x=%.2f, y=%.2f\n" % (person['position'][0], person['position'][1])
                    cv.Circle(im, (int(offset[0] + person['position'][0] * x_scale), \
                                int(offset[1] + person['position'][1] * y_scale)), 5, cv.RGB(255, 0, 0), 2)
            if count == 0:
                latex += "\\item[] No persons found.\n"
            latex += "\\end{enumerate}\n"

            latex += "\\subsection{Persons: GUIDED}\n"
            latex += "Marked as \\textbf{orange} in the map.\n"
            latex += "\\begin{enumerate}\n"
            count = 0
            for person in self.__topic_dict['person']:
                if person['value'] == 'OK_GUIDE':
                    count += 1
                    latex += "    \\item Location: x=%.2f, y=%.2f\n" % (person['position'][0], person['position'][1])
                    cv.Circle(im, (int(offset[0] + person['position'][0] * x_scale), \
                                int(offset[1] + person['position'][1] * y_scale)), 5, cv.RGB(255, 200, 0), 2)
            if count == 0:
                latex += "\\item[] No persons found.\n"
            latex += "\\end{enumerate}\n"

            latex += "\\subsection{Persons: OK}\n"
            latex += "Marked as \\textbf{pink} in the map.\n"
            latex += "\\begin{enumerate}\n"
            count = 0
            for person in self.__topic_dict['person']:
                if person['value'] == 'OK_EXIT':
                    count += 1
                    latex += "    \\item Location: x=%.2f, y=%.2f\n" % (person['position'][0], person['position'][1])
                    cv.Circle(im, (int(offset[0] + person['position'][0] * x_scale), \
                                int(offset[1] + person['position'][1] * y_scale)), 5, cv.RGB(255, 0, 200), 2)
            if count == 0:
                latex += "\\item[] No persons found.\n"
            latex += "\\end{enumerate}\n"
        else:
            latex = "No persons found, sorry."
            
        path = os.path.join(self.work, "arena_result.png")
        time.sleep(2)
        cv.SaveImage(path, im)

        return latex

    def produce_pdf(self):
        if not self.__last_latex:
            self.produce_latex()

        texfile_path = os.path.join(self.work, "emergency_report_%s" % time.strftime("%Y-%m-%d"))
        temp_path = texfile_path + ".pdf"
        final_path = os.path.join(self.location, "borg_emergency_report_%s" % time.strftime("%Y-%m-%d") + ".pdf")
        texfile = open(texfile_path + ".tex", 'w')
        texfile.write(self.__last_latex)
        texfile.close()
        subprocess.Popen("cd %s && pdflatex %s" % (self.work, texfile_path), shell=True).wait()
        subprocess.Popen("cd %s && pdflatex %s" % (self.work, texfile_path), shell=True).wait()
        shutil.copy2(temp_path, final_path)
        print "Copied to %s" % final_path
    
        self.__last_report = final_path
        return texfile_path

    def open_pdf(self):
        if self.__last_report:
            subprocess.Popen("gnome-open %s" % self.__last_report, shell=True)
        else:
            print "No PDF file available yet"


if __name__ == "__main__":


    brain.setup_logging(logging.getLogger(logging_namespace), None, None, "DEBUG")
    report = Report("/media/44C6-03F2/emergency_report/")

    person_value_option_list = ['NOT_OK', 'OK_EXIT', 'OK_GUIDE']
    for i in range(100):
        if random.random() < 0.5:
            report.add("hum", random.random() * 100)
        if random.random() < 0.5:
            report.add("temp", random.random() * 100 - 50)
        if random.random() < 0.2:
            report.add("person", person_value_option_list[random.randint(0, len(person_value_option_list) - 1)], \
                    position = (7 - random.random() * 8, 5 - random.random() * 10))
    print report.produce_latex()
    report.produce_pdf()
    report.open_pdf()

