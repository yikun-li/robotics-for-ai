import pycurl
import cStringIO as StringIO
from alsaaudio import *
import time
import wave
import logging
import urllib
import multiprocessing

import nullhandler
import loggingextra


class _AsyncWrapper(multiprocessing.Process):
    def __init__(self, tts, text):
        super(_AsyncWrapper, self).__init__()
        self.tts = tts
        self.text = text
    
    def run(self):
        self.tts.say(self.text, async=False)

"""
The MaryTTS class exposes the Text-to-Speech system of
OpenMary to Python, using the HTTP webservice of MaryTTS.
It provides functions to get a list of voices, synthesize
text and play the resulting Wave File.
"""
class MaryTTS(object):
    """
    Set up some default options, a default voice and logging
    """
    def __init__(self):
        self.__host = "localhost"
        self.__port = 59125
        self.__voice = 'dfki-prudence-hsmm'
        self.__voice_selections = 'dfki-prudence-hsmm en_GB female hmm'
        self.__logger = logging.getLogger('Borg.Brain.Util.MaryTTS')
        self.__logger.addHandler(nullhandler.NullHandler())
        self.__available_voices = None
        self.__set_default_parameters()
        self.__request_content_type = None

    """
    Set up the host and port of the OpenMary TTS server. By default,
    the host is assumed to be localhost and the port 59125
    """
    def set_host(self, host, port=59125):
        self.__host = host
        self.__port = port

    """
    Helper function to get the proper URL to a HTTP service on the
    OpenMary TTS HTTP server
    """
    def _get_url(self, service):
        return "http://%s:%s/%s" % (self.__host, self.__port, service)

    """
    Set the default parameters for the OpenMary HTTP request to synthesize
    """
    def __set_default_parameters(self):
        self.__parameters = {
            'AUDIO': 'WAVE_FILE',
            'AUDIO_OUT': 'WAVE_FILE',
            'HELP_TEXT': '',
            'INPUT_TEXT': '',
            'INPUT_TYPE': 'TEXT',
            'LOCALE': 'en_GB',
            'OUTPUT_TEXT': '',
            'OUTPUT_TYPE': 'AUDIO',
            'VOICE': self.__voice,
            'VOICE_SELECTIONS': self.__voice_selections,
            'effect_Chorus_default': 'Default',
            'effect_Chorus_help': 'Help',
            'effect_Chorus_parameters': 'delay1:466;amp1:0.54;delay2:600;amp2:-0.10;delay3:250;amp3:0.30',
            'effect_Chorus_selected': '',
            'effect_F0Add_default': 'Default',
            'effect_F0Add_help': 'Help',
            'effect_F0Add_parameters': 'f0Add:50.0;',
            'effect_F0Add_selected': '',
            'effect_F0Scale_default': 'Default',
            'effect_F0Scale_help': 'Help',
            'effect_F0Scale_parameters': 'f0Scale:2.0;',
            'effect_F0Scale_selected': '',
            'effect_FIRFilter_default': 'Default',
            'effect_FIRFilter_help': 'Help',
            'effect_FIRFilter_parameters': 'type:3;fc1:500.0;fc2:2000.0',
            'effect_FIRFilter_selected': '',
            'effect_JetPilot_default': 'Default',
            'effect_JetPilot_help': 'Help',
            'effect_JetPilot_parameters': '',
            'effect_JetPilot_selected': '',
            'effect_Rate_default': 'Default',
            'effect_Rate_help': 'Help',
            'effect_Rate_parameters': 'durScale:1.5;',
            'effect_Rate_selected': '',
            'effect_Robot_default': 'Default',
            'effect_Robot_help': 'Help',
            'effect_Robot_parameters': 'amount:100.0;',
            'effect_Robot_selected': '',
            'effect_Stadium_default': 'Default',
            'effect_Stadium_help': 'Help',
            'effect_Stadium_parameters': 'amount:100.0',
            'effect_Stadium_selected': '',
            'effect_TractScaler_default': 'Default',
            'effect_TractScaler_help': 'Help',
            'effect_TractScaler_parameters': 'amount:1.5;',
            'effect_TractScaler_selected': '',
            'effect_Volume_default': 'Default',
            'effect_Volume_help': 'Help',
            'effect_Volume_parameters': 'amount:2.0;',
            'effect_Volume_selected': '',
            'effect_Whisper_default': 'Default',
            'effect_Whisper_help': 'Help',
            'effect_Whisper_parameters': 'amount:100.0;',
            'effect_Whisper_selected': '',
        }

    """
    A function wrapping the Curl request to the OpenMary HTTP server. It accesses
    the service on the configured server with the provided parameters. When
    as_string is set to True, the result is returned as a string, otherwise the
    result will be returned as the cStringIO object it was read to
    """
    def _perform_request(self, service, parameters={}, as_string=True):
        output = StringIO.StringIO()
        curl = pycurl.Curl()
        curl.setopt(pycurl.URL, self._get_url(service))
        curl.setopt(pycurl.WRITEFUNCTION, output.write)

        # Add parameters as POST parameters when provided
        if parameters:
            curl.setopt(pycurl.POST, 1)
            curl.setopt(pycurl.POSTFIELDS, urllib.urlencode(parameters))

        # Try to perform the request
        try:
            curl.perform()
            self.__request_content_type = curl.getinfo(pycurl.CONTENT_TYPE)
        except pycurl.error, e:
            # Request failed. Try to provide intelligent feedback
            code, msg = e
            if 7 == code:
                self.__logger.error("Could not connect to MaryTTS on %s. Is MaryTTS HTTP server running?" % self.url)
            else:
                self.__logger.error("Request to MaryTTS failed. %s (%d)" % (code, msg))
            output.close()
            return None

        # Return the cStringIO object when requested
        if not as_string:
            return output

        # Extract the string value
        val = output.getvalue()
        output.close()
        return val

    """
    This method will return a list of voices that are installed
    in the MaryTTS server
    """
    def get_voices(self):
        if self.__available_voices is not None:
            return self.__available_voices.keys()

        response = self._perform_request("voices")
        if not response:
            self.__available_voices = {}
            return []
            
        self.__available_voices = {}
        voices = response.split("\n")
        for voice in voices:
            if not voice:
                continue
            parts = voice.split(" ")
            name = parts[0]
            self.__available_voices[name] = voice

        return self.__available_voices.keys()

    """
    A wrapper function to change the configured voice without doing
    any checks. This method should be only used internally after validation,
    and is used here to avoid code duplication.
    """
    def __set_voice(self, name):
        self.__voice = name
        self.__voice_selections = self.__available_voices[name]
        self.__parameters['VOICE'] = self.__voice
        self.__parameters['VOICE_SELECTIONS'] = self.__voice_selections
        self.__logger.debug("Set voice to %s" % name)

    """
    Set the voice that will be used for synthesizing text. The string should
    be the name of the voice, either the full name or just the actual name part.
    E.g., for fcki-prudence-hmms, both fcki-prudence-hmms and prudence are accepted.
    All names are normalized to lowercase first.
    """
    def set_voice(self, voice):
        voice = voice.lower()
        avail = self.get_voices()

        # Try to match with short voice names, following the pattern
        # <supplier>-<name>(-algorithm)
        if not voice in avail:
            for avail_voice in avail:
                parts = avail_voice.split("-")
                if len(parts) > 1 and voice == parts[1]:
                    voice = avail_voice
                    break
                
        # Check if the voice is available / installed
        if not voice in avail:
            if self.__voice: # A voice as already configured
                self.__logger.error(
                    "Voice %s is not available in MaryTTS setup. " \
                    "Sticking with %s" % (voice, self.__voice)
                )
            elif avail: # No voice configured yet, just pick one
                self.__set_voice(avail.keys()[0])
                self.__logger.error(
                    "Voice %s is not available in MaryTTS setup. " \
                    "Falling back to %s" % self.__voice
                )
            else: # No voices available at all
                self.__logger.error(
                    "No voices available in MaryTTS setup. " \
                    "Please check the installation"
                )
            return

        # Voice is available, select it
        self.__set_voice(voice)
        
    """
    Say the specified text: wraps the TTS function and the play_wave function
    """
    def say(self, text, async=True):
        if async:
            p = _AsyncWrapper(self, text)
            p.start()
            return

        text = text.strip()
        self.__logger.debug("Saying: %s" % text)
        output = self.synthesize(text)
        if output:
            try:
                wavefile = wave.open(output)
            except wave.Error, e:
                self.__logger.error("MaryTTS produced non-wave output. Cannot say text.")
                return

            self._play_wave(wavefile)
            wavefile.close()
            output.close()
        else:
            self.__logger.error("Synthesizing text failed. Cannot say text.")

    """
    Access the OpenMary TTS web server to synthesize the string and return it as
    a wave file. The output is an instance of the cStringIO class which can be
    played as a WAVE file
    """
    def synthesize(self, text):
        parameters = self.__parameters
        parameters['INPUT_TEXT'] = text
        
        output = self._perform_request("process", parameters, as_string=False)
        if not output:
            return None

        output.seek(0)
        if self.__request_content_type != "audio/x-wav":
            self.__logger.error(
                "Content type of response to synthesize request was not " \
                "audio/x-wav, but %s. Something went wrong." \
                % self.__request_content_type
            )
            self.__logger.debug(output.getvalue())
            output.close()
            return None

        return output

    """
    Play an wave.wave_read object using alsaaudio
    """
    def _play_wave(self, wavestream):
        pcm = PCM(type=PCM_PLAYBACK, mode=PCM_NORMAL, card='default')
        pcm.setchannels(wavestream.getnchannels())
        pcm.setrate(wavestream.getframerate())
        pcm.setformat(PCM_FORMAT_S16_LE)
        pcm.write(wavestream.readframes(1000000))

"""
Example script
"""
t = MaryTTS()
if __name__ == "__main__":
    logging.getLogger('Borg.Brain').addHandler(loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.DEBUG)
    import random

    while True:
        voices = t.get_voices()
        voice = voices[random.randint(0, len(voices) - 1)]
        t.set_voice(voice)
        print "Selected voice %s" % voice
        print "Please enter what you would like me to say"
        txt = raw_input()
        if "quit" == txt:
            break
        t.say(txt)
