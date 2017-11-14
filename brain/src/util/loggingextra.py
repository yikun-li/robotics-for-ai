import logging
import sys

def ScreenOutput(logFormat = None):
    return get_stream_handler(format=logFormat)

def FileOutput(filename, format = None):
    handler = logging.FileHandler(filename)
    handler.setFormatter(get_formatter(format))
    return handler

def get_stream_handler(output = None, format = None):
    if not output:
        output = sys.stdout
    handler = logging.StreamHandler(output)
    handler.setFormatter(get_formatter())
    return handler
    
def get_formatter(format = None):
    if not format:
        format = "[%(asctime)s][%(name)s] %(levelname)s: %(message)s"
    return logging.Formatter(format)
