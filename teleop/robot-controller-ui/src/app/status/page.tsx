'use client';

// import { useEffect, useState } from 'react';
// import ros from '@/services/ros'; // Update path if needed

export default function RosStatusPage() {
  // const [status, setStatus] = useState('Status');

  // useEffect(() => {
  //   ros.on('connection', () => {
  //     setStatus('✅ Connected');
  //   });

  //   ros.on('error', (error: Error) => {
  //     setStatus(`❌ Error: ${error.message}`);
  //   });

  //   ros.on('close', () => {
  //     setStatus('⚠️ Connection closed');
  //   });
  // }, []);

  return (
    <main className="min-h-screen p-8 sm:p-16 bg-[#0e0e0e] text-white font-mono">
      <div className="max-w-4xl mx-auto flex flex-col gap-6">
        <h1 className="text-3xl font-bold border-b border-gray-700 pb-2">
          ROS Bridge Status
        </h1>

        <div className="text-xl text-green-400">
          <strong>Status:</strong> {status}
        </div>

        {/* <div className="mt-6 p-4 bg-black/50 rounded-lg overflow-auto text-sm text-gray-300 border border-gray-700 max-h-[400px]">
          <pre>{JSON.stringify(ros, null, 2)}</pre>
        </div> */}
      </div>
    </main>
  );
}
