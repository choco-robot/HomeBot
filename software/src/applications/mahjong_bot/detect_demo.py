# import the inference-sdk
from inference_sdk import InferenceHTTPClient

# initialize the client
CLIENT = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="alxKvvMcxuKoen6CB9BT"
)

# infer on a local image
result = CLIENT.infer("test1.jpg", model_id="mahjong-vtacs-mexax-m4vyu-sjtd-rojrz/1")

