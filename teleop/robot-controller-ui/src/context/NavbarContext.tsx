'use client';

import { createContext, useContext, useState, ReactNode } from 'react';

const NavbarContext = createContext<{
  isVisible: boolean;
  toggleNavbar: () => void;
}>({
  isVisible: true,
  toggleNavbar: () => {},
});

export const NavbarProvider = ({ children }: { children: ReactNode }) => {
  const [isVisible, setIsVisible] = useState(true);
  const toggleNavbar = () => setIsVisible((prev) => !prev);

  return (
    <NavbarContext.Provider value={{ isVisible, toggleNavbar }}>
      {children}
    </NavbarContext.Provider>
  );
};

export const useNavbar = () => useContext(NavbarContext);