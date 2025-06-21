import { RosStatus } from "../components/RosStatus";

function Status() {
    return (
        <div>
            <h2 style={{ height: "100%" }}>Status Page</h2>
            <p>Will display all the connection information and will also probably contains connection to ROS2</p>
            <RosStatus />
        </div>
    );

}

export default Status;