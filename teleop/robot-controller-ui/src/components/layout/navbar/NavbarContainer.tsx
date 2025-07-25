const NavbarContainer = ({ children }: { children: React.ReactNode }) => (
  <div className="flex justify-center items-center px-[2%] py-4 bg-[#24252A] relative resize-y">
    <nav className="w-full flex items-center relative flex-col md:flex-row">
      {children}
    </nav>
  </div>
);

export default NavbarContainer;
