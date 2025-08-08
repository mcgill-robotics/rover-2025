import Link from 'next/link';

const NavLink = ({ label }: { label: string }) => {
  const path = label.toLowerCase() === 'status' ? '/' : `/${label.toLowerCase()}`;
  return (
    <li className="font-medium text-[20px] text-[#f8f8f8] mx-[60px] hover:text-red-600 hover:underline active:text-blue-700 my-2 md:my-0 transition-colors duration-300">
      <Link href={path} title={`${label} Page`}>
        {label}
      </Link>
    </li>
  );
};

export default NavLink;
