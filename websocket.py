# Written by Leia Hannes, Hannah Loly, Chris Rogers
# Center for Engineering Education and Outreach
# Summer of 2025

import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String
import json

import asyncio
import websockets
import ssl
import json
import threading
import signal
import sys

from rclpy.executors import MultiThreadedExecutor

uri = "wss://chrisrogers.pyscriptapps.com/talking-on-a-channel/api/channels/hackathon"

class WebSocketPublisherNode(Node):
    """Node that publishes incoming WebSocket messages to ROS topics"""
    
    def __init__(self):
        super().__init__('websocket_publisher_node')
        queue_size = 10
        
        # Publisher for incoming WebSocket data
        self.publisher_ = self.create_publisher(String, 'websocket/incoming', queue_size)
        
        # Publisher for connection status
        self.status_publisher_ = self.create_publisher(String, 'websocket/status', queue_size)
        
        self.get_logger().info('WebSocket Publisher Node initialized')

    def publish_websocket_message(self, data):
        """Publish incoming WebSocket data to ROS topic"""
        try:
            msg = String()
            msg.data = json.dumps(data)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published WebSocket message: {data.get("topic", "unknown")}')
        except Exception as e:
            self.get_logger().error(f'Error publishing WebSocket message: {e}')

    def publish_status(self, status):
        """Publish WebSocket connection status"""
        try:
            msg = String()
            msg.data = status
            self.status_publisher_.publish(msg)
            self.get_logger().info(f'WebSocket status: {status}')
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

class WebSocketSubscriberNode(Node):
    """Node that subscribes to ROS topics and sends messages via WebSocket"""
    
    def __init__(self):
        super().__init__('websocket_subscriber_node')
        queue_size = 10
        
        # Subscriber for outgoing messages
        self.subscription = self.create_subscription(
            String,
            'websocket/outgoing',
            self.outgoing_message_callback,
            queue_size
        )
        
        # Subscriber for WebSocket commands (connect, disconnect, etc.)
        self.command_subscription = self.create_subscription(
            String,
            'websocket/command',
            self.command_callback,
            queue_size
        )
        
        # WebSocket client reference
        self.websocket_client = None
        self.get_logger().info('WebSocket Subscriber Node initialized')

    def set_websocket_client(self, client):
        """Set reference to WebSocket client"""
        self.websocket_client = client

    def outgoing_message_callback(self, msg):
        """Handle ROS messages that should be sent to WebSocket"""
        try:
            if self.websocket_client and self.websocket_client.is_connected():
                # Parse the ROS message data
                try:
                    message_data = json.loads(msg.data)
                except json.JSONDecodeError:
                    # If not JSON, send as simple message
                    message_data = {
                        'type': 'ros_message',
                        'topic': 'ros/outgoing',
                        'value': msg.data,
                        'timestamp': self.get_clock().now().to_msg()._sec
                    }
                
                # Send to WebSocket
                asyncio.run_coroutine_threadsafe(
                    self.websocket_client.send_message(message_data),
                    self.websocket_client.loop
                )
                self.get_logger().info(f'Queued message for WebSocket: {message_data}')
            else:
                self.get_logger().warn('WebSocket not connected - message not sent')
        except Exception as e:
            self.get_logger().error(f'Error handling outgoing message: {e}')

    def command_callback(self, msg):
        """Handle WebSocket commands (connect, disconnect, etc.)"""
        try:
            command = msg.data.lower()
            self.get_logger().info(f'Received WebSocket command: {command}')
            
            if command == 'reconnect' and self.websocket_client:
                asyncio.run_coroutine_threadsafe(
                    self.websocket_client.reconnect(),
                    self.websocket_client.loop
                )
        except Exception as e:
            self.get_logger().error(f'Error handling command: {e}')

class WebSocketClient:
    """WebSocket client that communicates with both ROS nodes"""
    
    def __init__(self, publisher_node, subscriber_node):
        self.uri = uri
        self.publisher_node = publisher_node
        self.subscriber_node = subscriber_node
        self.websocket = None
        self.running = False
        self.loop = None
        self.send_queue = asyncio.Queue()
        self._connected = False
        
    def is_connected(self):
        """Check if WebSocket is connected"""
        if not self._connected or not self.websocket:
            return False
        if hasattr(self.websocket, 'closed'):
            return not self.websocket.closed
        return True
        #return self._connected and self.websocket and not self.websocket.closed
        
    async def send_message(self, message):
        """Queue a message to be sent to WebSocket"""
        if self.running:
            await self.send_queue.put(message)
        
    async def reconnect(self):
        """Reconnect to WebSocket server"""
        self.get_logger().info("Reconnecting to WebSocket...")
        self.running = False
        if self.websocket:
            await self.websocket.close()
        await asyncio.sleep(2)  # Wait before reconnecting
        await self.connect_and_run()
        
    async def connect_and_run(self):
        """Main WebSocket connection and message handling"""
        self.loop = asyncio.get_event_loop()
        
        try:
            self.publisher_node.publish_status("connecting")
            
            async with websockets.connect(self.uri, ssl=True) as websocket:
                self.websocket = websocket
                self.running = True
                self._connected = True
                
                self.publisher_node.publish_status("connected")
                print("WebSocket connected successfully")
                
                # Send initial message
                initial_message = {'topic': '/fred/image', 'value': 9}
                
                await websocket.send(json.dumps(initial_message))
                print(f"Sent initial message: {initial_message}")
                
                # Run send and receive tasks concurrently
                await asyncio.gather(
                    self.send_handler(),
                    self.receive_handler(),
                    return_exceptions=True
                )
                
        except Exception as e:
            self.publisher_node.publish_status(f"error: {str(e)}")
            print(f"WebSocket error: {e}")
        finally:
            self._connected = False
            self.running = False
            self.publisher_node.publish_status("disconnected")

    async def send_handler(self):
        """Handle outgoing messages from ROS to WebSocket"""
        while self.running and self.is_connected():
            try:
                # Wait for messages to send
                message = await asyncio.wait_for(self.send_queue.get(), timeout=1.0)
                await self.websocket.send(json.dumps(message))
                print(f"Sent to WebSocket: {message}")
            except asyncio.TimeoutError:
                #print("Send handler timeout (no messages in queue)")
                continue
            except Exception as e:
                print(f"Send handler error: {e}")
                break

    async def receive_handler(self):
        """Handle incoming WebSocket messages"""
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    
                    # Filter and process messages
                    if data.get('type') == 'data' and data.get('payload'):
                        payload = json.loads(data['payload'])
                        if '/URarm' in payload['topic']:  #'heartbeat' not in payload.get('topic', ''):
                            # Send to publisher node
                            self.publisher_node.publish_websocket_message(payload)
                            print(f'Received from WebSocket: {payload.get("value")}')
                    
                except json.JSONDecodeError:
                    print(f"Non-JSON message received: {message}")
                    # Still publish as raw text
                    self.publisher_node.publish_websocket_message({'raw_message': message})
                except Exception as e:
                    print(f"Error processing message: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed")
        except Exception as e:
            print(f"Receive handler error: {e}")

def run_websocket_client(publisher_node, subscriber_node, stop_event):
    """Run WebSocket client in asyncio event loop"""
    async def websocket_main():
        client = WebSocketClient(publisher_node, subscriber_node)
        subscriber_node.set_websocket_client(client)
        
        try:
            while not stop_event.is_set():
                await client.connect_and_run()
                if not stop_event.is_set():
                    print("WebSocket disconnected, attempting to reconnect in 5 seconds...")
                    await asyncio.sleep(5)
        except Exception as e:
            print(f"WebSocket client error: {e}")
        finally:
            print("WebSocket client stopped")
    
    # Create new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(websocket_main())
    finally:
        loop.close()

def start_websocket():
    """Main function with two separate nodes"""
    # Initialize ROS2
    rclpy.init()
    
    # Create both nodes
    publisher_node = WebSocketPublisherNode()
    subscriber_node = WebSocketSubscriberNode()
    
    # Create executor with appropriate number of threads
    # 2 threads: one for each node's callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)
    
    # Event for coordinating shutdown
    stop_event = threading.Event()
    
    # Set up signal handler for graceful shutdown
    def signal_handler(signum, frame):
        print("\nShutdown signal received...")
        stop_event.set()
        executor.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start WebSocket client in a separate thread
    websocket_thread = threading.Thread(
        target=run_websocket_client,
        args=(publisher_node, subscriber_node, stop_event),
        daemon=True
    )
    websocket_thread.start()
    
    try:
        print("Starting ROS2 MultiThreadedExecutor with WebSocket Publisher and Subscriber nodes...")
        print("Topics:")
        print("  - websocket/incoming: Incoming WebSocket messages")
        print("  - websocket/outgoing: Send messages to WebSocket")
        print("  - websocket/status: Connection status updates")
        print("  - websocket/command: WebSocket commands (reconnect, etc.)")
        
        # This will block and handle all ROS2 callbacks in multiple threads
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    finally:
        print("Shutting down...")
        stop_event.set()
        
        # Clean shutdown
        if websocket_thread.is_alive():
            websocket_thread.join(timeout=5)
            
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete")


class WebSubscriber(Node):

    def __init__(self):
        super().__init__("test_sub")
        self.subscription = self.create_subscription(String, '/websocket/incoming', self.callback, 10)
        self.states = None
    
    def callback(self, msg):
        self.states = msg.data
        self.done = True

    def read(self):
        self.done = False
        start = time.monotonic()
        while not self.done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > 0.1:
                return None
            time.sleep(0.1)
        self.done = False
        return self.states


class WebPublisher(Node):

    def __init__(self):
        super().__init__('websocket_message_sender')
        self.publisher_ = self.create_publisher(String, 'websocket/outgoing', 10)
        self.states = None

    def send(self, topic, value):
        message = {
            'type': 'ros_message',
            'topic': topic,
            'value': value,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        msg = String()
        msg.data = json.dumps(message)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published to websocket/outgoing: {msg.data}')


class Channel_Communication(Node):
    def __init__(self):
        self.sub = WebSubscriber()
        self.pub = WebPublisher()

    def read_data(self):
        return self.sub.read()

    def send_data(self, topic, value):
        self.pub.send(topic, value)

    def clear_queue(self):
        for i in range(10):
            self.read_data()
