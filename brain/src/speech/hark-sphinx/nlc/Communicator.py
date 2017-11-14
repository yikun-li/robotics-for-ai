import sys
import re
from pprint import pprint

from JSGFParser import JSGFParser
from DecoderAnalyzer import DecoderAnalyzer

class Communicator():

    def __init__(self, grammarFile):
       self.grammar = JSGFParser(grammarFile)
       self.stopWords = self.grammar.findStopwords() #optional words that don't add core meaning to the sentence
       self.reset()

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
            print(result['first']['difference_token'])
            understoodOptions = [result['first']['difference_token']['text'], result['second']['difference_token']['text']] # the two different understood options
            allOptions = self.grammar.findVariableItems(var, includeVars=False) # all the available options for this var from the grammar

            if var == "<start>":
                responseSentence = "Ask: who do you mean? %s " % ' or '.join(understoodOptions)
                repeat = "part"
                print ("listen for: %s" % str(allOptions))
            elif var == "<object>":
                responseSentence = "Ask: which object do you mean? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("listen for: %s" % str(allOptions))
            elif var == "<verb>":
                responseSentence = "Ask: what do you want me to do? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("listen for: %s" % str(allOptions))
            elif var == "<location>":
                responseSentence = "Ask: where do you mean? %s" % ' or '.join(understoodOptions)
                repeat = "part"
                print ("listen for: %s" % str(allOptions))
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

        # check for sentences
        if ("what is your teams name" in sentence):
            print("My teams name is BORG")
            return
        if ("what is your favorite color" in sentence):
            print("Black as the night")
            return

        # check for requests
        if ("follow me" in sentence):
            print ("Ok I will follow you")
            return
        if ("enter the elevator" in sentence):
            print("Ok I will go in the elevator")
            return
        if ("give me a drink" in sentence):
            print ("I will give you a drink")
            return
        if ("pick up the cup" in sentence):
            print ("I will pick up the cup")
            return
        
        # specific color cup
        regex = re.compile(ur'pick up the (.+) cup')
        matches = re.findall(regex, sentence)
        if matches:
            color = matches[0]
            print("I will pick up the %s cup" % color)
            return

        # order
        actions = self.parseOrderSentence(sentence)
        if actions:
            pprint(actions)
            return

    def parseOrderSentence(self, sentence):
        # remove the start of the sentence
        starts = self.grammar.findVariableItems("<start>", includeVars=False)
        for start in starts:
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

            action = {}
            action['verb'] = lastOrderVerb
            items = orders.split(" and ")
            action['items'] = items
            action['location'] = location
            actions.append(action)


        # Build a sentence to say for the robot
        first = True
        for action in actions:
            if first:
                response = "Ok "
            else:
                response = "then "

            response += "I will %s " % action['verb']
            firstItem = True
            for item in action['items']:
                if firstItem is False:
                    response += "and "
                response += "the %s " % item
                firstItem = False

            response += "to the %s" % location
        
            first = False

            print response
        return actions
            













if __name__ == '__main__':
    grammarFile = "grammar/experiment.gram"
    communicator = Communicator(grammarFile)
    analyzer = DecoderAnalyzer(grammarFile)
    # result = analyzer.analyze2Best("pseudo pick up the cup", "pseudo pick up the red cup")
    # result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice place chocolates to bedroom")
    # result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice get chocolates to bedroom")
    # result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice deliver chocolates to kitchen")
    result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "pseudo deliver chocolates to bedroom")
    # result = analyzer.analyze2Best("alice deliver ice tea to bedroom", "alice deliver chocolates to bedroom")
    # result = analyzer.analyze2Best("pseudo deliver ice tea to bedroom", "alice deliver chocolates to bedroom")
    response = communicator.resultResponse(result)
    pprint(response)

    # sentence = "pick sup the red cup"
    # sentence = "alice please bring the orange juice and the ice tea and the coffee to table one and the beer and the chocolats to the kitchen and take the peanuts and the chewing gums to the bedroom"
    # communicator.sentenceResponse(sentence)

