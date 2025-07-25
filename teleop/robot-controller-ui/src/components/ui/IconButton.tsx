import { ButtonHTMLAttributes } from 'react';

type IconButtonProps = ButtonHTMLAttributes<HTMLButtonElement> & {
  icon: React.ReactNode;
  className?: string;
};

export default function IconButton({ icon, className, ...props }: IconButtonProps) {
  return (
    <button
      className={`p-2 rounded-md transition-colors ${className}`}
      {...props}
    >
      {icon}
    </button>
  );
}
