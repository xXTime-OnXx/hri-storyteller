#!/usr/bin/env python3
import os
import requests
import rospy
import json
import re

class LLMService:
    """Handles all LLM interactions for story generation"""
    
    def __init__(self, model_name="gemini-2.5-flash:generateContent"):
            self.api_key = os.environ.get("GENAI_API_KEY")
            if not self.api_key:
                raise Exception("GENAI_API_KEY environment variable not set")
            
            self.model_name = model_name
            self.url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_name}"
            rospy.loginfo(f"GENAI_API_KEY loaded from environment")
            rospy.loginfo(f"Using Gemini model: {self.model_name}")
    

    def request(self, text):
        headers = {
            "x-goog-api-key": self.api_key,
            "Content-Type": "application/json"
        }
        payload = self.build_payload(text)
        rospy.loginfo(f"Gemini request body: {json.dumps(payload)}")

        try:
            res = requests.post(self.url, headers=headers, json=payload)
        except requests.RequestException as e:
            rospy.logerr(f"Request failed: {e}")
            raise

        if res.status_code != 200:
            rospy.logerr(f"Request to Gemini failed: {res.status_code} {res.text}")
            raise Exception(f"Request to Gemini failed: {res.status_code} {res.text}")

        data = res.json()
        try:
            print('gemini response')
            print(data)
            print('gemini text response')
            print(self.parse_gemini_json_response(data['candidates'][0]['content']['parts'][0]['text']))
            return self.parse_gemini_json_response(data['candidates'][0]['content']['parts'][0]['text'])
        except (KeyError, IndexError):
            rospy.logerr(f"Unexpected response structure: {json.dumps(data)}")
            raise

    
    def build_payload(self, text):
        # TODO: add system instructions to request
        return {
            "contents": [
                {
                    "parts": [
                        {"text": text}
                    ]
                }
            ]
        }

    def parse_gemini_json_response(self, text: str) -> dict:
        """
        Parse JSON from Gemini response that may be wrapped in markdown code fences.
        
        Args:
            text: Raw text from Gemini that may contain ```json\n...\n```
            
        Returns:
            Parsed JSON as dictionary
        """
        # Remove markdown code fences (```json and ```)
        cleaned_text = re.sub(r'^```json\s*\n?', '', text.strip())
        cleaned_text = re.sub(r'\n?```\s*$', '', cleaned_text)
        
        # Parse the JSON
        return json.loads(cleaned_text)
    