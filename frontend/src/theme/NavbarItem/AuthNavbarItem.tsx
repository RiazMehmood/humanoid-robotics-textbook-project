/**
 * Custom NavbarItem component to display AuthButton in the navbar.
 */
import React from 'react';
import AuthButton from '../../components/AuthButton';

export default function AuthNavbarItem(): JSX.Element {
  return (
    <div style={{ display: 'flex', alignItems: 'center' }}>
      <AuthButton />
    </div>
  );
}

