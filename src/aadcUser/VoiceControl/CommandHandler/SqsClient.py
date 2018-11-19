#!/usr/bin/env python3
import boto3
import time


class SqsClient:

    is_initialized = False

    def init(self):
        # Create SQS client
        self.sqs = boto3.client(
            'sqs',
            region_name='eu-central-1',
            aws_access_key_id=ACCESS_KEY,
            aws_secret_access_key=SECRET_KEY,
            aws_session_token=SESSION_TOKEN,
        )
        self.is_initialized = True


    def getNextCommand(self):
        # Receive message from SQS queue
        response = self.sqs.receive_message(
            QueueUrl=SQS_URL,
            AttributeNames=[
                'SentTimestamp'
            ],
            MaxNumberOfMessages=1,
            MessageAttributeNames=[
                'All'
            ],
            VisibilityTimeout=0,
            WaitTimeSeconds=0
        )

        if('Messages' in response):
            message = response['Messages'][0]
            receipt_handle = message['ReceiptHandle']
            # Delete received message from queue
            self.sqs.delete_message(
                QueueUrl=SQS_URL,
                ReceiptHandle=receipt_handle
            )
            print('Received and deleted message: %s' % message['Body'])
            return message['Body']
        else: 
            print("No messages available")
            return False



if __name__ == '__main__':

    poller = SqsClient()
    poller.init()


    while(True):
        poller.getNextCommand()
        time.sleep(2)
    