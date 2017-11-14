import sys
from pprint import pprint
from collections import Counter
import math
import time
import operator
from collections import OrderedDict
import difflib

from JSGFParser import JSGFParser

class DecoderAnalyzer:

    def __init__(self, grammarFile=None):
        if grammarFile:
            self.setGrammar(grammarFile)

    def setGrammar(self, grammarFile):
        self.grammar = JSGFParser(grammarFile)
        self.stopWords = self.grammar.findStopwords() #optional words that don't add core meaning to the sentence


    def getNBestList(self, decoder, n=None):
        nBestList = {}
        for oneBest in decoder.nbest():
            if not oneBest.hyp() is None:
                score = oneBest.hyp().best_score # log scale. http://stackoverflow.com/questions/20825654/cmusphinx-what-is-the-score-of-a-recognised-hypothesis#comment31476787_20867454
                hypstr = oneBest.hyp().hypstr
                hypstr = self.filterHypstr(hypstr) # remove words that don't add core meaning

                if not hypstr in nBestList:
                    # nBestList[hypstr] = {
                    #     'count': 0,
                    #     'score': 0,
                    #     'norm_score': 0
                    # }
                    nBestList[hypstr] = 0
                nBestList[hypstr] += 1
                # nBestList[hypstr]['count'] += 1
                # nBestList[hypstr]['score'] += score

        decodersBest = self.filterHypstr(decoder.hyp().hypstr) # best hypothesis
        nBestList[decodersBest] = 999999 # make sure that the decoders best has the best rank always


        nBestListOrdered = list(OrderedDict(sorted(nBestList.items(), key=lambda t: t[1], reverse=True)))

        if n:
            nBestListOrdered = nBestListOrdered[0:n]

        # sorted_nBestList = sorted(nBestList.items(), key=operator.itemgetter(1))
        # pprint(nBestList[0:2])

        # calculate the normalized score for each found sentence
        # for hypstr in nBestList:
        #     nBestList[hypstr]['norm_score'] = nBestList[hypstr]['score'] / nBestList[hypstr]['count']

        # nBestCounter = Counter(nBestList)
        # order list
        # take n best

        return nBestListOrdered

    def filterHypstr(self, hypstr):
        """
           filter the given string by taking out the words in the stopWords list
        """
        for stopWord in self.stopWords:
            hypstr = hypstr.replace(" %s " % stopWord, " ")

        return hypstr

    def bestResult(self, nBestList):

        bestResult = None
        for hypstr in nBestList:
            if not bestResult:
                bestResult = hypstr
                continue

            if nBestList[hypstr]['norm_score'] > nBestList[bestResult]['norm_score']:
                bestResult = hypstr

        return (bestResult, nBestList[bestResult])

    def difference(self, first, second):
        diff =  difflib.ndiff(first.split(), second.split())

        return diff


    def analyze2Best(self, firstBest, secondBest):

        # return {
        #         "first": {
        #             "sentence": "alice follow me",
        #             "difference_token": {"text": "alice", "tag": "robot_alice", "var": "<start>"}
        #             },
        #         "second": {
        #             "sentence": "pseudo follow me",
        #             "difference_token": {"text": "pseudo", "tag": "robot_sudo", "var": "<start>"}
        #             },
        #         "same_var": "<start>",
        #         "same_tag": None,
        #         "accepted_string": None
        #     }


        if not secondBest:
            print("[DecoderAnalyzer] No second best found, accepting first and only understood sentence")
            return {
                "first": {
                    "sentence": firstBest,
                    "difference_token": None
                },
                "second": None,
                "same_var": None,
                "same_tag": None,
                "accepted_string": firstBest
            }

        diff = self.difference(firstBest, secondBest)
        first = []
        second = []
        for a, b in enumerate(diff):
            if b[0]=='-':
                first.append(b[2:])
            if b[0]=='+': 
                second.append(b[2:])

        firstTokenString = ' '.join(first)
        firstToken = self.grammar.findToken(firstTokenString)
        secondTokenString = ' '.join(second)
        secondToken = self.grammar.findToken(secondTokenString)

        acceptedString = None
        sameVar = None
        sameTag = None

        if not firstToken and not secondToken: # @TODO check if maybe many difference and that's why no token is found...
            if (not firstTokenString in firstBest) or (not secondTokenString in secondBest):
                print ("[DecoderAnalyzer] multiple differences between strings at different locations")
            else:
                print ("[DecoderAnalyzer]")
                print (firstBest)
                print (secondBest)
                print ("[DecoderAnalyzer] No significant differences found. Accept first result")
                acceptedString = firstBest
        elif firstToken and secondToken:
            print ("[DecoderAnalyzer] Found token difference between the strings")
            if firstToken["var"] == secondToken["var"]:
                sameVar = firstToken["var"]
                print("[DecoderAnalyzer] with the same var")
                if (firstToken["tag"] and secondToken["tag"] and (firstToken["tag"] == secondToken["tag"])):
                    sameTag = firstToken["tag"]
                    acceptedString = firstBest
                    print("[DecoderAnalyzer] with the same tag")
                else:
                    print("[DecoderAnalyzer] with different tags")
            else:
                print("[DecoderAnalyzer] with different vars")
        else:
            differentToken = firstToken if firstToken else secondToken
            print ("[DecoderAnalyzer] Different Token: ")
            print(differentToken)


        result = {
                "first": {
                    "sentence": firstBest,
                    "difference_token": firstToken
                    },
                "second": {
                    "sentence": secondBest,
                    "difference_token": secondToken
                    },
                "same_var": sameVar,
                "same_tag": sameTag,
                "accepted_string": acceptedString
            }


        return result





    def analyze(self, decoder):
        if decoder.hyp() is None:
            return None

        decodersBest = decoder.hyp().hypstr # best hypothesis
        decodersBest = self.filterHypstr(decodersBest)


        decoder.get_lattice().write("experiment/lattice_output/" + str(time.time()) + ".lattice")


        nBestList = self.getNBestList(decoder=decoder, n=2)
        print("")
        pprint(nBestList)
        if len(nBestList) == 1:
            acceptedString = nBestList[0]
        else:
            analyze2BestResult = self.analyze2Best(nBestList[0], nBestList[1])
            pprint(analyze2BestResult)
            acceptedString = analyze2BestResult["accepted_string"]


        return acceptedString

if __name__ == '__main__':
    analyzer = DecoderAnalyzer("grammar/experiment.gram")
    # result = analyzer.analyze2Best("Pseudo pick up the cup", "Pseudo pick up the red cup")
    result = analyzer.analyze2Best("alice deliver chocolates to bedroom", "alice place chocolates to bedroom")
    pprint(result)