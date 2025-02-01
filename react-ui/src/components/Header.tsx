import Navbar from "./Navbar";

interface HeaderProps {
    title: string;
    width: string;
    height: string;
    border?: string;
}

function Header({ title, width, height, border }: HeaderProps) {
    return (
        <div style={{ height, width, border }}>
            <h1>{title}</h1>
            <Navbar />
        </div>
    );
}

export default Header;