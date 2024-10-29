import { useEffect, useState } from "react";
import ros from "../services/ros";

export function RosStatus() {
    const [status, setStatus] = useState("Status");

    useEffect(() => {
        ros.on("connection", () => {
            setStatus("successful");
        });

        ros.on("error", (error: Error) => {
            setStatus(`errored out (${error})`);
        });

        ros.on("close", () => {
            setStatus("closed");
        });
    }, []);

    return (
        <div>
            <h1>{status}</h1>
            <div>{JSON.stringify(ros, null, 2)}</div>
        </div>
    );
}