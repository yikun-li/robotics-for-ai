import sys
import re
from pprint import pprint

from JSGFParser import JSGFParser
from DecoderAnalyzer import DecoderAnalyzer

class Conversator():
    grammar = None
    stopWords = None

    def __init__(self, grammarFile=None):
        print ("[Conversator] initializing...")
        self.reset()
        if grammarFile:
            self.setGrammar(grammarFile)

    def setGrammar(self, grammarFile):
        print ("[Conversator] Grammar file set: %s" % grammarFile)
        self.grammar = JSGFParser(grammarFile)
        self.stopWords = self.grammar.findStopwords() #optional words that don't add core meaning to the sentence


    def reset(self):
        self.status = "idle"

    def filterStopwords(self, sentence):
        """
           filter the given string by taking out the words in the stopWords list
        """

        for stopWord in self.stopWords:
            sentence = sentence.replace(" %s " % stopWord, " ")

        return sentence


    def resultResponse(self, result):

        # @TODO remove this
        # return {
        #     "repeat": "part",
        #     "understoodOptions": ['alice', 'pseudo'],
        #     "allOptions": ['alice', 'pseudo', 'rick', 'smith'],
        #     "responseSentence": "Who do you mean? Alice or Pseudo."
        # }

        understoodOptions = None
        allOptions = None

        # we already found a string that we consider as understood correctly
        if result['accepted_string']:
            responseSentence = "I understood '%s'" % result['accepted_string']
            repeat = "no"

        # we understood something different. The difference is not within the same var
        elif result['same_var'] is None:
            responseSentence = "I didn't understand. Please repeat your sentence."
            repeat = "all"

        else:
            var = result['same_var']
            understoodOptions = [result['first']['difference_token']['text'], result['second']['difference_token']['text']] # the two different understood options
            allOptions = self.grammar.findVariableItems(var, includeVars=False) # all the available options for this var from the grammar

            if var == "<name>":
                responseSentence = "Who do you mean? %s " % ' or '.join(understoodOptions)
                repeat = "part"
                print ("[Conversator] listen for: %s" % str(allOptions))
            elif var == "<object>":
                responseSentence = "Which object do you mean? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("[Conversator] listen for: %s" % str(allOptions))
            elif var == "<verb>":
                responseSentence = "What do you want me to do? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("[Conversator] listen for: %s" % str(allOptions))
            elif var == "<location>":
                responseSentence = "Where do you mean? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("[Conversator] listen for: %s" % str(allOptions))
            elif var == "<color>":
                responseSentence = "Which color do you mean? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("[Conversator] listen for: %s" % str(allOptions))
            else:
                responseSentence = "I didn't understand. Please repeat your sentence."
                repeat = "all"

        response = {
            "repeat": repeat,
            "understoodOptions": understoodOptions,
            "allOptions": allOptions,
            "responseSentence": responseSentence
        }

        return response

    def sentenceResponse(self, sentence, filterStopwords=True):
        if filterStopwords:
            sentence = self.filterStopwords(sentence)

        actions = []

        # check for sentences
        if ("what is your teams name" in sentence):
            actions.append({"type": "say", "content": "My teams name is BORG"})
            return actions
        elif ("what is your favorite color" in sentence):
            actions.append({"type": "say", "content": "Black as the night"})
            return actions

        # check for requests
        elif ("follow me" in sentence):
            actions.append({"type": "say", "content": "Ok I will follow you"})
            return actions
        elif ("enter the elevator" in sentence):
            actions.append({"type": "say", "content": "Ok I will go in the elevator"})
            return actions
        elif ("give me a drink" in sentence):
            actions.append({"type": "say", "content": "I will give you a drink"})
            return actions
        elif ("go duck yourself" in sentence):
            actions.append({"type": "say", "content": "Just like I did your mom?"})
            return actions

        
        # specific color cup
        regex = re.compile(ur'pick up (.+) cup')
        matches = re.findall(regex, sentence)

        if matches:
            color = matches[0]
            content = "I will pick up the %s cup" % color
            actions.append({"type": "say", "content": content})
            actions.append({"type": "do", "content": {'verb': 'pickup', 'items': ["%s cup" % color], 'location': None}})
            return actions

        # order
        actions = self.parseOrderSentence(sentence)
        if actions:
            return actions

        return []

    def parseOrderSentence(self, sentence):
        # remove the start of the sentence
        names = self.grammar.findVariableItems("<name>", includeVars=False)
        for name in names:
            start = "okay %s" % name
            if  sentence.startswith(start):
                sentence = sentence.lstrip(start)

        # split on different locations. these will be the different orders
        locations = self.grammar.findVariableItems("<location>", includeVars=False)
        regex = re.compile(" to (%s)" % '|'.join(locations))
        matches = re.split(regex, sentence)
        if not matches:
            return None


        # make pairs for each order and location couple
        order_locations = []
        matchCount = len(matches)
        for idx in range(0, matchCount, 2):
            if idx + 1 >= matchCount:
                break
            order_locations.append((matches[idx].strip().lstrip("and").lstrip(), matches[idx+1].strip()))

        # build actions from the order location pairs.
        # each order parts is analyzed (find the verb and different items)
        # each action is a dict indicating the task verb, items and location
        verbs = self.grammar.findVariableItems("<verb>", includeVars=False)
        lastOrderVerb = None
        actions = []
        for order_location in order_locations:
            orders = order_location[0]
            location = order_location[1]
            for verb in verbs:
                if verb == orders[0:len(verb)]:
                    lastOrderVerb = verb
                    orders = orders[len(verb) + 1:]
                    break

            actionContent = {}
            actionContent['verb'] = lastOrderVerb
            items = orders.split(" and ")
            actionContent['items'] = items
            actionContent['location'] = location
            actions.append({"type": "do", "content": actionContent})


        # Build a sentence to say for the robot
        first = True
        response = ""
        for action in actions:
            actionContent = action['content']
            if first:
                response += "Ok "
            else:
                response += " then "

            response += "I will %s " % actionContent['verb']
            firstItem = True
            for item in actionContent['items']:
                if firstItem is False:
                    response += "and "
                response += "the %s " % item
                firstItem = False

            response += "to the %s" % actionContent['location']
        
            first = False

        actions.insert(0, {"type": "say", "content": response}) # add as first action

        return actions


    def getRepeatAction(self, sentence=None):
        if not sentence:
            sentence = "I didn't understand. Please repeat your sentence."
        return {"type": "say", "content": sentence}
            













if __name__ == '__main__':
    grammarFile = "grammar/experiment.gram"
    conversator = Conversator(grammarFile)
    analyzer = DecoderAnalyzer(grammarFile)
    # result = analyzer.analyze2Best("pseudo pick up the cup", "pseudo pick up the red cup")
    # result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice place chocolates to bedroom")
    # result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice get chocolates to bedroom")
    # result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice deliver chocolates to kitchen")
    result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "pseudo deliver chocolates to bedroom")
    # result = analyzer.analyze2Best("alice deliver ice tea to bedroom", "alice deliver chocolates to bedroom")
    # result = analyzer.analyze2Best("pseudo deliver ice tea to bedroom", "alice deliver chocolates to bedroom")
    response = conversator.resultResponse(result)
    pprint(response)

    # sentence = "pick sup the red cup"
    # sentence = "alice please bring the orange juice and the ice tea and the coffee to table one and the beer and the chocolats to the kitchen and take the peanuts and the chewing gums to the bedroom"
    # conversator.sentenceResponse(sentence)

