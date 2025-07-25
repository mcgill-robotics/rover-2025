'use client';
import { useNavbar } from '@/context/NavbarContext';
import { Menu } from 'lucide-react';
import IconButton from '@/components/ui/IconButton';

export default function BurgerButton() {
  const { toggleNavbar } = useNavbar();

  return (
    <IconButton
      onClick={toggleNavbar}
      icon={<Menu size={24} />}
      aria-label="Open navbar"
      className="fixed top-4 right-4 z-50 bg-[#24252A] text-white hover:bg-white hover:text-black"
    />
  );
}
