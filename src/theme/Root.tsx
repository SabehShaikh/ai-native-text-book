import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

// Root component wrapper - renders on all pages
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
