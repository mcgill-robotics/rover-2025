// index.js

const rclnodejs = require('rclnodejs');

async function main() {
  // Initialize rclnodejs
  await rclnodejs.init();

  // Create a ROS2 node named 'chatter_publisher_node'
  const node = rclnodejs.createNode('chatter_publisher_node');

  // Create a publisher for std_msgs/msg/String on the '/chatter' topic
  const publisher = node.createPublisher('std_msgs/msg/String', 'chatter');

  // Create a message object
  const StringMsg = rclnodejs.require('std_msgs/msg/String');
  const message = new StringMsg();

  // Initialize a counter
  let count = 0;

  // Function to publish messages periodically
  const publishMessage = () => {
    message.data = `Hello, ROS2! Count: ${count}`;
    publisher.publish(message);
    console.log(`Published: "${message.data}"`);
    count += 1;
  };

  // Publish a message every second
  setInterval(publishMessage, 1000);

  // Start spinning the node
  rclnodejs.spin(node);
}

main().catch((err) => {
  console.error(`Error: ${err}`);
});
