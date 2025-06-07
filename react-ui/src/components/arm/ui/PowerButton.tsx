import React, { useState } from 'react';
import './styles/PowerButton.css';

interface PowerButtonProps {
  onClick?: () => void;
  size?: 'small' | 'medium' | 'large';
  isActive?: boolean;
  label?: string;
}

const PowerButton: React.FC<PowerButtonProps> = ({
  onClick,
  size = 'medium',
  isActive = false,
  label = 'POWER'
}) => {
  const [active, setActive] = useState(isActive);

  const handleClick = () => {
    setActive(!active);
    if (onClick) onClick();
  };

  return (
    <div className="power-button-container">
      <button 
        className={`power-button ${size} ${active ? 'active' : ''}`}
        onClick={handleClick}
        aria-label="Power button"
      >
        <div className="power-button-inner">
          <svg
            className="power-icon"
            xmlns="http://www.w3.org/2000/svg"
            width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round"
          >
            <line x1="12" y1="2" x2="12" y2="12" />
            <path d="M17.657 6.343a8 8 0 1 1-11.314 0" />
          </svg>
          <div className="power-label">{label}</div>
        </div>
      </button>
    </div>
  );
};

export default PowerButton;