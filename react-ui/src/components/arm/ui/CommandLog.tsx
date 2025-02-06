const CommandLog: React.FC<{ commands: string[] }> = ({ commands }) => {
  return (
    <div className="log">
      <h2>Command Log</h2>
      {commands.map((cmd, index) => (
        <div key={index}>{cmd}</div>
      ))}
    </div>
  );
};

export default CommandLog;