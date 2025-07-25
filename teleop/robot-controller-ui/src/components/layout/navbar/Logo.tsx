'use client';

import Image from 'next/image';
import logo from '@/assets/images/logo.png';
import { useNavbar } from '@/context/NavbarContext';

const Logo = () => {
  const { toggleNavbar } = useNavbar();

  return (
    <button onClick={toggleNavbar} className="ml-2">
      <Image
        src={logo}
        alt="Logo"
        className="w-[50px] h-auto cursor-pointer"
      />
    </button>
  );
};

export default Logo;
