import NavLink from './NavLink';

const navItems = ['Status', 'Control', 'Dashboard'];

const NavLinks = () => (
  <ul className="list-none m-0 p-0 flex absolute left-1/2 -translate-x-1/2 flex-col md:flex-row static:md:static md:transform-none">
    {navItems.map((item) => (
      <NavLink key={item} label={item} />
    ))}
  </ul>
);

export default NavLinks;

