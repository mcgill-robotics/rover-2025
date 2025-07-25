'use client';

import { useNavbar } from '@/context/NavbarContext';
import NavbarContainer from './NavbarContainer';
import NavLinks from './NavLinks';
import Logo from './Logo';
import BurgerButton from './BurgerButton';

export default function Navbar() {
  const { isVisible } = useNavbar();

  return (
    <>
      {/* Show burger menu only when navbar is hidden */}
      {!isVisible && <BurgerButton />}

      {/* Render navbar when visible */}
      {isVisible && (
        <NavbarContainer>
          <div className="flex w-full items-center justify-between">
            <Logo /> {/* Now used to close the navbar */}
          </div>
          <NavLinks />
        </NavbarContainer>
      )}
    </>
  );
}
